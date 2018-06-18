#Copyright 2018, David Dias
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np

from _bag_msg import *
from _std_msgs import *
import quaternion_tools

class VecAccel(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.linear = VecVector3(messages, False)
        self.angular = VecVector3(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.linear.write_message(msg.linear, t, count)
        self.angular.write_message(msg.angular, t, count)
        
class VecAccelStamped(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.accel = VecAccel(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.accel.write_message(msg.accel, t, count)
        
class VecAccelWithCovariance(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.accel = VecAccel(messages, False)
        self.covariance = np.zeros((36, messages), dtype=np.float64)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.accel.write_message(msg.accel, t, count)
        self.covariance[:, count] = msg.covariance

class VecAccelWithCovarianceStamped(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.accel = VecAccelWithCovariance(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.accel.write_message(msg.accel, t, count)
        
class VecInertia(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.m = np.zeros(messages, dtype=np.float64)
        self.com = VecVector3(messages, False)
        self.ixx = np.zeros(messages, dtype=np.float64)
        self.ixy = np.zeros(messages, dtype=np.float64)
        self.ixz = np.zeros(messages, dtype=np.float64)
        self.iyy = np.zeros(messages, dtype=np.float64)
        self.iyz = np.zeros(messages, dtype=np.float64)
        self.izz = np.zeros(messages, dtype=np.float64)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.m[count] = msg.m
        self.com.write_message(msg.com, t, count)
        self.ixx[count] = msg.ixx
        self.ixy[count] = msg.ixy
        self.ixz[count] = msg.ixz
        self.iyy[count] = msg.iyy
        self.iyz[count] = msg.iyz
        self.izz[count] = msg.izz
        
class VecInertiaStamped(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.inertia = VecInertia(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.inertia.write_message(msg.inertia, t, count)

class VecPoint(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.x = np.zeros(messages, dtype=np.float64)
        self.y = np.zeros(messages, dtype=np.float64)
        self.z = np.zeros(messages, dtype=np.float64)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.x[count] = msg.x
        self.y[count] = msg.y
        self.z[count] = msg.z
    
    def get_point_x(self):
        return self.x
    
    def get_point_y(self):
        return self.y
    
    def get_point_z(self):
        return self.z
    
    def set_point_x(self, point_x):
        self.x = point_x
    
    def set_point_y(self, point_y):
        self.y = point_y
    
    def set_point_z(self, point_z):
        self.z = point_z
    
    def write_rosbag_msg(self, msg, msg_idx):
        msg.x = self.x[msg_idx]
        msg.y = self.y[msg_idx]
        msg.z = self.z[msg_idx]
        
class VecPoint32(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.x = np.zeros(messages, dtype=np.float32)
        self.y = np.zeros(messages, dtype=np.float32)
        self.z = np.zeros(messages, dtype=np.float32)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.x[count] = msg.x
        self.y[count] = msg.y
        self.z[count] = msg.z
    
    def get_point_x(self):
        return self.x
    
    def get_point_y(self):
        return self.y
    
    def get_point_z(self):
        return self.z
    
    def set_point_x(self, point_x):
        self.x = point_x
    
    def set_point_y(self, point_y):
        self.y = point_y
    
    def set_point_z(self, point_z):
        self.z = point_z
    
    def write_rosbag_msg(self, msg, msg_idx):
        msg.x = self.x[msg_idx]
        msg.y = self.y[msg_idx]
        msg.z = self.z[msg_idx]
        
class VecPointStamped(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.point = VecPoint(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.point.write_message(msg.point, t, count)
        
class VecPose(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.position = VecPoint(messages, False)
        self.orientation = VecQuaternion(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.position.write_message(msg.position, t, count)
        self.orientation.write_message(msg.orientation, t, count)
        
    def get_position_x(self):
        return self.position.get_point_x()
    
    def get_position_y(self):
        return self.position.get_point_y()
    
    def get_position_z(self):
        return self.position.get_point_z()
    
    def get_quat(self):
        return self.orientation.get_quat()
    
    def set_position_x(self, position_x):
        return self.position.set_point_x(position_x)
    
    def set_position_y(self, position_y):
        return self.position.set_point_y(position_y)
    
    def set_position_z(self, position_z):
        return self.position.set_point_z(position_z)
    
    def set_quat(self, quat):
        return self.orientation.set_quat(quat)
    
    def write_rosbag_msg(self, msg, msg_idx):
        self.position.write_rosbag_msg(msg.position, msg_idx)
        self.orientation.write_rosbag_msg(msg.orientation, msg_idx)
        
class VecPose2D(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.x = np.zeros(messages, dtype=np.float64)
        self.y = np.zeros(messages, dtype=np.float64)
        self.theta = np.zeros(messages, dtype=np.float64)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.x[count] = msg.x
        self.y[count] = msg.y
        self.theta[count] = msg.theta

class VecPoseStamped(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.pose = VecPose(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.pose.write_message(msg.pose, t, count)
        
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
    
    def get_roll(self):
        q = self.get_quat()
        return np.degrees(quaternion_tools.quat2roll(q))

    def get_pitch(self):
        q = self.get_quat()
        return np.degrees(quaternion_tools.quat2pitch(q))

    def get_yaw(self):
        q = self.get_quat()
        return np.degrees(quaternion_tools.quat2yaw(q))
    
class VecPoseWithCovariance(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.pose = VecPose(messages, False)
        self.covariance = np.zeros((36, messages), dtype=np.float64)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.pose.write_message(msg.pose, t, count)
        self.covariance[:, count] = msg.covariance
        
    def get_position_x(self):
        return self.pose.get_position_x()
    
    def get_position_y(self):
        return self.pose.get_position_y()
    
    def get_position_z(self):
        return self.pose.get_position_z()
    
    def get_quat(self):
        return self.pose.get_quat()
    
    def set_position_x(self, position_x):
        return self.pose.set_position_x(position_x)
    
    def set_position_y(self, position_y):
        return self.pose.set_position_y(position_y)
    
    def set_position_z(self, position_z):
        return self.pose.set_position_z(position_z)
    
    def set_quat(self, quat):
        return self.pose.set_quat(quat)
    
    def write_rosbag_msg(self, msg, msg_idx):
        self.pose.write_rosbag_msg(msg.pose, msg_idx)
        msg.covariance = self.covariance[:, msg_idx].tolist()

class VecPoseWithCovarianceStamped(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.pose = VecPoseWithCovariance(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.pose.write_message(msg.pose, t, count)

class VecQuaternion(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.quat = np.ndarray(shape=(4, messages), dtype=np.float64)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.quat[:,count] = [msg.x, msg.y, msg.z, msg.w]
        
    def get_quat(self):
        return self.quat
    
    def set_quat(self, quat):
        self.quat = quat
        
    def write_rosbag_msg(self, msg, msg_idx):
        msg.x = self.quat[0, msg_idx]
        msg.y = self.quat[1, msg_idx]
        msg.z = self.quat[2, msg_idx]
        msg.w = self.quat[3, msg_idx]

class VecQuaternionStamped(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.quaternion = VecQuaternion(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.quaternion.write_message(msg.quaternion, t, count)
        
class VecTransform(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.translation = VecVector3(messages, False)
        self.rotation = VecQuaternion(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.translation.write_message(msg.translation, t, count)
        self.rotation.write_message(msg.rotation, t, count)
        
class VecTransformStamped(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.child_frame_id = []
        self.transform = VecTransform(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.child_frame_id.append(msg.child_frame_id)
        self.transform.write_message(msg.transform, t, count)
        
class VecTwist(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.linear = VecVector3(messages, False)
        self.angular = VecVector3(messages, False)
      
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.linear.write_message(msg.linear, t, count)
        self.angular.write_message(msg.angular, t, count)
        
    def write_rosbag_msg(self, msg, msg_idx):
        self.linear.write_rosbag_msg(msg.linear, msg_idx)
        self.angular.write_rosbag_msg(msg.angular, msg_idx)

class VecTwistStamped(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.twist = VecTwist(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.twist.write_message(msg.twist, t, count)
        
class VecTwistWithCovariance(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.twist = VecTwist(messages, False)
        self.covariance = np.zeros((36, messages), dtype=np.float64)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.twist.write_message(msg.twist, t, count)
        self.covariance[:, count] = msg.covariance
        
    def write_rosbag_msg(self, msg, msg_idx):
        self.twist.write_rosbag_msg(msg.twist, msg_idx)
        msg.covariance = self.covariance[:, msg_idx].tolist()
        
class VecTwistWithCovarianceStamped(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.twist = VecTwistWithCovariance(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.twist.write_message(msg.twist, t, count)

class VecVector3(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.x = np.zeros(messages, dtype=np.float64)
        self.y = np.zeros(messages, dtype=np.float64)
        self.z = np.zeros(messages, dtype=np.float64)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.x[count] = msg.x
        self.y[count] = msg.y
        self.z[count] = msg.z
        
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y
    
    def get_z(self):
        return self.z
    
    def write_rosbag_msg(self, msg, msg_idx):
        msg.x = self.x[msg_idx]
        msg.y = self.y[msg_idx]
        msg.z = self.z[msg_idx]

class VecVector3Stamped(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.vector = VecVector3(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.vector.write_message(msg.vector, t, count)

class VecWrench(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.force = VecVector3(messages, False)
        self.torque = VecVector3(messages, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.force.write_message(msg.force, t, count)
        self.torque.write_message(msg.torque, t, count)
        
class VecWrenchStamped(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.wrench = VecWrench(messages, False)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.wrench.write_message(msg.wrench, t, count)
        
