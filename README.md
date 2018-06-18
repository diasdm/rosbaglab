# rosbaglab

This is a Python library written with the intent to automatize ROS Bags plotting, specially plots of variables with physical meaning, e.g. distances, velocities, orientations, etc. Yet, it is also possible to edit, save and create ROS Bags.

rosbaglab vectorizes a ROS Bag, meaning that (almost) every field of a ROS topic is in the same array. This allows easy processing of topic data, e.g. to calculate position errors.

## Features
* Plot ROS Bags, even data from different topics can be plotted in the same graph if they have the same physical meaning, e.g. position from Odometry and PoseWithCovarianceStamped. 
* Edit ROS Bags, if you want e.g. to remove a some topics from the ROS Bag while making sure that everything else remains the same.
* Create ROS Bags, if you need to create and save a ROS Bag for some specific purpose.

## Suported messages
* All std_msgs, geometry_msgs, nav_msgs and sensor_msgs, except the ones which use non fixed sized arrays, e.g. sensor_msgs/LaserScan.

## Getting Started

TODO write instruction on how to (install?) the library.

### Prerequisites

To use this library you need to install rospy, numpy and yaml.

```
pip install rospy numpy yaml
```

## Contributing

If you code something that might be useful for someone else let us know by opening an issue / pull request. If we agree that it is general enough we will include it in the library.

## Authors

* **David Dias**

See also the list of [contributors](CONTRIBUTORS.md) who participated in this project.

## License

This project is licensed under the 3-Clause BSD License - see the [LICENSE](LICENSE) file for details
