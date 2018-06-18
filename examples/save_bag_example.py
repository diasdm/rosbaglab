#!/usr/bin/env python
from context import topic_tools
from context import topic_plots
from context import bag_tools

if __name__ == '__main__':
    # To run this example ypu should download this bag from http://download.ros.org/data/amcl/
    # and place it in the examples folder
    # Bag to be loaded
    bag_to_load = 'texas_willow_hallway_loop_indexed.bag'
    # Loads a bag
    bag_obj = bag_tools.load_bag(bag_to_load)
    # Saves bag under a different name 
    bag_tools.save_bag(bag_obj, 'someNewName.bag')
