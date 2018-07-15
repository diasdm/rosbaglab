#Copyright 2018, David Dias
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np
import nav_msgs.msg

from _bag_msg import *
from _std_msgs import *
from _geometry_msgs import *
import quaternion_tools

class VecMapMetaData(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.map_load_time = VecTime(messages, False)
        self.resolution = VecBuiltInType(messages, np.float32, False)
        self.width = VecBuiltInType(messages, np.uint32, False)
        self.height = VecBuiltInType(messages, np.uint32, False)
        self.origin = VecPose(messages, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.map_load_time.write_message(msg.map_load_time, t, count)
        self.resolution.write_message(msg.resolution, t, count)
        self.width.write_message(msg.width, t, count)
        self.height.write_message(msg.height, t, count)
        self.origin.write_message(msg.origin, t, count)

    def write_ros_msg(self, msg, msg_idx):
        self.map_load_time.write_ros_msg(msg.map_load_time, msg_idx)
        msg.resolution = self.resolution[msg_idx]
        msg.width = self.width[msg_idx]
        msg.height = self.height[msg_idx]
        self.origin.write_ros_msg(msg.origin, msg_idx)

    def get_ros_msg(self, msg_idx):
        msg = nav_msgs.msg.MapMetaData()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

class VecOdometry(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.child_frame_id = []
        self.pose = VecPoseWithCovariance(messages, False)
        self.twist = VecTwistWithCovariance(messages, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.child_frame_id.append(msg.child_frame_id)
        self.pose.write_message(msg.pose, t, count)
        self.twist.write_message(msg.twist, t, count)

    def write_ros_msg(self, msg, msg_idx):
        self.header.write_ros_msg(msg.header, msg_idx)
        msg.child_frame_id = self.child_frame_id[msg_idx]
        self.pose.write_ros_msg(msg.pose, msg_idx)
        self.twist.write_ros_msg(msg.twist, msg_idx)

    def get_ros_msg(self, msg_idx):
        msg = nav_msgs.msg.Odometry()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

    def get_position_x(self):
        return self.pose.get_position_x()

    def get_position_y(self):
        return self.pose.get_position_y()

    def get_position_z(self):
        return self.pose.get_position_z()

    def get_quat(self):
        return self.pose.get_quat()

    def get_time(self):
        return self.header.get_time()

    def set_position_x(self, position_x):
        return self.pose.set_position_x(position_x)

    def set_position_y(self, position_y):
        return self.pose.set_position_y(position_y)

    def set_position_z(self, position_z):
        return self.pose.set_position_z(position_z)

    def set_quat(self, quat):
        return self.pose.set_quat(quat)

    def set_time(self, ft):
        return self.header.set_time(ft)

    def get_roll(self):
        q = self.get_quat()
        return np.degrees(quaternion_tools.quat2roll(q))

    def get_pitch(self):
        q = self.get_quat()
        return np.degrees(quaternion_tools.quat2pitch(q))

    def get_yaw(self):
        q = self.get_quat()
        return np.degrees(quaternion_tools.quat2yaw(q))

