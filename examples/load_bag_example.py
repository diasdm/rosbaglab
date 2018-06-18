#!/usr/bin/env python
from context import topic_tools
from context import topic_plots
from context import bag_tools

if __name__ == '__main__':
    # To run this example ypu should download this bag from http://download.ros.org/data/amcl/
    # and place it in the examples folder
    # Bag to be loaded
    bag_to_load = 'texas_willow_hallway_loop_indexed.bag'
    # Load ros bag
    bag_obj = bag_tools.load_bag(bag_to_load)
    # Print some info about the ros bags
    print(bag_obj)
    # Plot all topics using the standart functions
    bag_obj.plot_all_topics()
