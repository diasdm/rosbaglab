# ROSBagLab vectorized messages 

In rosbaglab all messages are vectorized, this makes ploting and processing topic data very easy.

## Message structure

All vectorized messages extend the same base class, VecBagMsg. This class contains the only thing all ROS messages recorded have in common, a time of recording.

## Standard message functions

In order to make the automatic plot of topics possible some functions where standardized. There are some obvious functions messing such as the ones for linear velocity, this is simply because they wheren't needed so far.

#### Time
* `get_time(self)`
#### Position
* `get_position_x(self, position_x)`
* `get_position_y(self, position_y)`
* `get_position_z(self, position_z)`
#### Orientation
* `get_roll(self):`
* `get_pitch(self):`
* `get_yaw(self):`
#### Angular velocity
* `get_angular_velocity_x(self)`
* `get_angular_velocity_y(self)`
* `get_angular_velocity_z(self)`
#### Linear acceleration
* `get_linear_acceleration_x(self)`
* `get_linear_acceleration_y(self)`
* `get_linear_acceleration_z(self)`
## Adding your custom message

In order to add your custom message you need to:
* Add the respective class in the (_custom_msgs.py)[_custom_msgs.py] file.
* Add the message type and correspondent class in the (_msgs.py)[_msgs.py] file.

If you have any questions fell free to open an issue.
