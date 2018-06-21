#Copyright 2018, David Dias
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rosbag
import numpy as np
import yaml

import topic_tools
import topic_plots

class Bag:
    def __init__(self, path_to_bag = ''):
        self.path = path_to_bag
        self.diff_topic_num = 0
        self.topic_name_list = []
        # list of lists with idx by topic type
        self.diff_topic_idx = []
        self.diff_topic_types = []
        self.topics_list = topic_tools.TopicList()

    def __str__(self):
        return 'Bag ' + self.path + ' contains ' + str(len(self.topics_list)) + ' topics from ' + str(self.diff_topic_num) + ' different types.'

    def add_topic(self, topic, i):
        type_idx = 0
        # if the topic type is in the bag properties
        for diff_topic_type in self.diff_topic_types:
            if diff_topic_type == topic.type:
                self.diff_topic_idx[type_idx].append(i)
                self.topic_name_list.append(topic.topic)
                self.topics_list.append(topic)
                return
            else:
                type_idx += 1

        # if the topic type is not in the bag properties
        self.diff_topic_types.append(topic.type)
        self.topics_list.append(topic)
        self.diff_topic_idx.append([])
        self.topic_name_list.append(topic.topic)
        self.diff_topic_idx[self.diff_topic_num].append(i)
        self.diff_topic_num += 1

    def get_topics(self, topic_names):
        out_list = []

        if isinstance(topic_names, list):
            for topic_name in topic_names:
                try:
                    i = self.topic_name_list.index(topic_name)
                except ValueError:
                    print(topic_name + ' is not in the topics_list')
                    return
                out_list.append(self.topics_list[i])
        else:
            try:
                i = self.topic_name_list.index(topic_names)
            except ValueError:
                print(topic_names + ' is not in the topics_list')
                return
            out_list.append(self.topics_list[i])

        return out_list

    def plot_all_topics(self):
        topic_plots.plot_topics_list(self.topics_list)

def obj_list_write_message(topic, msg, t, topic_list):
    for i in range(len(topic_list)):
        if topic == topic_list[i].topic:
            type = topic_list[i].type
            count = topic_list[i].count
            # write message to memory
            topic_list[i].obj.write_message(msg, t, count)
            topic_list[i].count += 1
            break

def load_bag(bag_to_load):
    input_dict = yaml.load(rosbag.Bag(bag_to_load, 'r')._get_yaml_info())

    bag_obj = Bag(bag_to_load)

    freq = -1.0

    for i in range(len(input_dict['topics'])):
        # Since there are topics that don't have a frequency field
        if 'frequency' in input_dict['topics'][i]:
            freq = input_dict['topics'][i]['frequency']
        else:
            freq = -1.0

        topic = topic_tools.Topic( freq, input_dict['topics'][i]['messages'],
                          input_dict['topics'][i]['topic'], input_dict['topics'][i]['type'])
        if topic.obj is None:
            print('Message object not found for message of type ' + topic.type + ' , skipping topic ' + topic.topic)
        else:
            bag_obj.add_topic(topic, i)

    for topic, msg, t in rosbag.Bag(bag_to_load).read_messages():
        obj_list_write_message(topic, msg, t, bag_obj.topics_list)

    return bag_obj

def load_multiple_bags(bags_to_load):
    bags_objs = []

    for bag_to_load in bags_to_load:
        bag_obj = load_bag(bag_to_load)
        bags_objs.append(bag_obj)

    return bags_objs

def merge_bags_by_reference(bags_objs):
    bag_obj = Bag('?')
    i = 0
    for obj in bags_objs:
        for topic in obj.topics_list:
            bag_obj.add_topic(topic, i)
            i += 1

    return bag_obj

def get_closest_time(bag_obj, topics_list, mintime, next_msg_to_retrieve):
    # assuming there aren't 2 equal times
    dtime = np.zeros(len(bag_obj.topics_list), dtype=np.float64)

    for topic_idx, topic in enumerate(topics_list):
        if next_msg_to_retrieve[topic_idx] < topic.messages:
            dtime[topic_idx] = topic.obj.bagtime.ft[next_msg_to_retrieve[topic_idx]] - mintime
        else:
            dtime[topic_idx] = np.inf

    idx = np.argmin(dtime)
    mintime = topics_list[idx].obj.bagtime.ft[next_msg_to_retrieve[idx]]
    return [mintime, idx]

def get_ros_msg(topics_list, idx, msg_idx):
    topic = topics_list[idx].topic
    try:
        [msg, t] = topics_list[idx].obj.get_ros_msg(msg_idx)
    except TypeError:
        msg = None
        t = None
        print('Topic type ' + topics_list[idx].obj.__class__.__name__ + ' does not have a get_ros_msg function!')
    return [topic, msg, t]

def save_bag(bag_obj, save_path):
    totalmsgs = 0

    for topic in bag_obj.topics_list:
        totalmsgs += topic.messages

    next_msg_to_retrieve = np.zeros(len(bag_obj.topics_list), dtype=np.uint32)

    with rosbag.Bag(save_path, 'w') as output_bag:
        mintime = -1
        while totalmsgs > 0:
            [mintime, idx] = get_closest_time(bag_obj, bag_obj.topics_list, mintime, next_msg_to_retrieve)
            [topic, msg, t] = get_ros_msg(bag_obj.topics_list, idx, next_msg_to_retrieve[idx])
            if msg != None:
                output_bag.write(topic, msg, t)
            next_msg_to_retrieve[idx] += 1
            totalmsgs -= 1
    return
