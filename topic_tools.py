#Copyright 2018, David Dias
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import numpy as np
from tf.transformations import *

import topic_plots
from vecmsgs import _msgs

class Topic:
    def __init__(self, frequency, messages, topic, type):
        self.frequency = frequency
        self.messages = messages
        self.topic = topic
        self.label = topic
        self.type = type
        self.count = 0
        self.obj = _msgs.get_message_obj(type, messages)
        
    def __str__(self):
        return 'Topic ' + self.topic + ' is of type ' + self.type + ' and contains ' + self.messages + ' messages.'
    
class TopicList(list):
    def plot_position_time_x(self):
        topic_plots.plot_position_time_x([0], self, range(len(self)))
        
    def plot_position_time_y(self):
        topic_plots.plot_position_time_y([0], self, range(len(self)))
    
    def plot_position_x_y(self):
        topic_plots.plot_position_x_y([0], self, range(len(self)))
     
    def plot_position(self):
        topic_plots.plot_position([0], self, range(len(self)))
        
    def plot_orientation_roll(self):    
        topic_plots.plot_orientation_roll([0], self, range(len(self)))
        
    def plot_orientation_pitch(self):    
        topic_plots.plot_orientation_pitch([0], self, range(len(self)))
        
    def plot_orientation_yaw(self):    
        topic_plots.plot_orientation_yaw([0], self, range(len(self)))
        
    def plot_orientation(self):    
        topic_plots.plot_orientation([0], self, range(len(self)))
    
    def append(self, item):
        if not isinstance(item, Topic):
            raise TypeError, 'item is not of type %s' % Topic.type
        super(TopicList, self).append(item)
    
    def print_topics_names(self):
        print('This list contains:')
        for topic in self:
            print(topic.topic + ' of type ' + topic.type)
