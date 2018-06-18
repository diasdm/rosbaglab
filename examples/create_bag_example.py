#!/usr/bin/env python
import time
import numpy as np

from context import topic_tools
from context import topic_plots
from context import bag_tools

if __name__ == '__main__':
    bag_obj = bag_tools.Bag()
    
    # For those how what to write their own bags
    
    # Make an odometry spiral example
    # Topic properties
    messages = 1000 # Message number
    frequency = 10 # Hz
    topic_name = '/odom'
    topic_type = 'nav_msgs/Odometry'
    # Create topic obj and add it to the bag
    topic_obj = topic_tools.Topic(frequency, messages, topic_name, topic_type)
    bag_obj.topics_list.append(topic_obj)
    # Time stamps
    t0 = time.time()
    ft = np.linspace(t0, t0 + messages * 1.0 / frequency, num=messages, dtype=np.float64)
    topic_obj.obj.set_time(ft)
    topic_obj.obj.set_bag_time(ft)
    # Compute spiral
    theta = np.linspace(0, 8 * np.pi, num=messages)
    r =  np.exp( -0.1 * np.linspace(0, 8 * np.pi, num=messages) )
    position_x = r * np.cos(theta)
    position_y = r * np.sin(theta)
    # Save position to topic
    topic_obj.obj.set_position_x(position_x)
    topic_obj.obj.set_position_y(position_y)
    # Plot the bag topic
    bag_obj.plot_all_topics()
    # Save the bag_obj
    bag_tools.save_bag(bag_obj, '/home/david/projects/atrvjr_catkin/bags/mocap_2imu/bag_2018-05-01-16-08-43-atrvjr-xpto.bag')
