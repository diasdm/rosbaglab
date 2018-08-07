#!/usr/bin/env python
from context import topic_tools
from context import topic_plots
from context import bag_tools

if __name__ == '__main__':
    # To run this example ypu should download this bag from http://download.ros.org/data/amcl/
    # and place it in the examples folder
    # Bag to be loaded
    bag_to_load = '/home/david/ros_ws/bags/outside/2018-08-04/2.bag'
    # Load ros bag
    bag_obj = bag_tools.load_bag(bag_to_load)
    # Print some info about the ros bags
    print(bag_obj)
    # Plot all topics using the standart functions
    bag_obj.plot_all_topics()
