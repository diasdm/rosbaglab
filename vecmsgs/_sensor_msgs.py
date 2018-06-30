#Copyright 2018, David Dias
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np
import sensor_msgs.msg
from rospy.rostime import Time

from _bag_msg import VecBagMsg
from _std_msgs import *
from _geometry_msgs import *
import quaternion_tools

class VecFluidPressure(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.fluid_pressure = VecBuiltInType(messages, np.float64, False)
        self.variance = VecBuiltInType(messages, np.float64, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.fluid_pressure.write_message(msg.fluid_pressure, t, count)
        self.variance.write_message(msg.variance, t, count)

    def write_ros_msg(self, msg, msg_idx):
        self.header.write_ros_msg(msg.header, msg_idx)
        self.fluid_pressure.write_ros_msg(msg.fluid_pressure, msg_idx)
        self.variance.write_ros_msg(msg.variance, msg_idx)

    def get_ros_msg(self, msg_idx):
        msg = sensor_msgs.msg.FluidPressure()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

class VecIlluminance(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.illuminance = VecBuiltInType(messages, np.float64, False)
        self.variance = VecBuiltInType(messages, np.float64, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.illuminance.write_message(msg.illuminance, t, count)
        self.variance.write_message(msg.variance, t, count)

    def write_ros_msg(self, msg, msg_idx):
        self.header.write_ros_msg(msg.header, msg_idx)
        self.illuminance.write_ros_msg(msg.illuminance, msg_idx)
        self.variance.write_ros_msg(msg.variance, msg_idx)

    def get_ros_msg(self, msg_idx):
        msg = sensor_msgs.msg.Illuminance()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

class VecImu(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.orientation = VecQuaternion(messages, False)
        self.orientation_covariance = np.zeros((9, messages), dtype=np.float64)
        self.angular_velocity = VecVector3(messages, False)
        self.angular_velocity_covariance = np.zeros((9, messages), dtype=np.float64)
        self.linear_acceleration = VecVector3(messages, False)
        self.linear_acceleration_covariance = np.zeros((9, messages), dtype=np.float64)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.orientation.write_message(msg.orientation, t, count)
        self.orientation_covariance[:, count] = msg.orientation_covariance
        self.angular_velocity.write_message(msg.angular_velocity, t, count)
        self.angular_velocity_covariance[:, count] = msg.angular_velocity_covariance
        self.linear_acceleration.write_message(msg.linear_acceleration, t, count)
        self.linear_acceleration_covariance[:, count] = msg.linear_acceleration_covariance

    def write_ros_msg(self, msg, msg_idx):
        self.header.write_ros_msg(msg.header, msg_idx)
        self.orientation.write_ros_msg(msg.orientation, msg_idx)
        msg.orientation_covariance = self.orientation_covariance[:, msg_idx]
        self.angular_velocity.write_ros_msg(msg.angular_velocity, msg_idx)
        msg.angular_velocity_covariance = self.angular_velocity_covariance[:, msg_idx]
        self.linear_acceleration.write_ros_msg(msg.linear_acceleration, msg_idx)
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance[:, msg_idx]

    def get_ros_msg(self, msg_idx):
        msg = sensor_msgs.msg.Imu()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

    def get_quat(self):
        return self.orientation.get_quat()

    def get_time(self):
        return self.header.get_time()

    def get_angular_velocity_x(self):
        return self.angular_velocity.get_x()

    def get_angular_velocity_y(self):
        return self.angular_velocity.get_y()

    def get_angular_velocity_z(self):
        return self.angular_velocity.get_z()

    def get_linear_acceleration_x(self):
        return self.linear_acceleration.get_x()

    def get_linear_acceleration_y(self):
        return self.linear_acceleration.get_y()

    def get_linear_acceleration_z(self):
        return self.linear_acceleration.get_z()

    def get_roll(self):
        q = self.get_quat()
        return np.degrees(quaternion_tools.quat2roll(q))

    def get_pitch(self):
        q = self.get_quat()
        return np.degrees(quaternion_tools.quat2pitch(q))

    def get_yaw(self):
        q = self.get_quat()
        return np.degrees(quaternion_tools.quat2yaw(q))

class VecJoyFeedback(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.type = VecBuiltInType(messages, np.uint8, False)
        self.id = VecBuiltInType(messages, np.uint8, False)
        self.intensity = VecBuiltInType(messages, np.float32, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.type.write_message(msg.type, t, count)
        self.id.write_message(msg.id, t, count)
        self.intensity.write_message(msg.intensity, t, count)

    def write_ros_msg(self, msg, msg_idx):
        self.type.write_ros_msg(msg.type, msg_idx)
        self.id.write_ros_msg(msg.id, msg_idx)
        self.intensity.write_ros_msg(msg.intensity, msg_idx)

    def get_ros_msg(self, msg_idx):
        msg = sensor_msgs.msg.JoyFeedback()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

class VecMagneticField(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.magnetic_field = VecVector3(messages, False)
        self.magnetic_field_covariance = np.zeros((9, messages), dtype=np.float64)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.magnetic_field.write_message(msg.magnetic_field, t, count)
        self.magnetic_field_covariance[:, count] = msg.magnetic_field_covariance

    def write_ros_msg(self, msg, msg_idx):
        self.header.write_ros_msg(msg.header, msg_idx)
        self.magnetic_field.write_ros_msg(msg.magnetic_field, msg_idx)
        msg.magnetic_field_covariance = self.magnetic_field_covariance[:, msg_idx]

    def get_ros_msg(self, msg_idx):
        msg = sensor_msgs.msg.MagneticField()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

class VecNavSatFix(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.status = VecNavSatStatus(messages, False)
        self.latitude = VecBuiltInType(messages, np.float64, False)
        self.longitude = VecBuiltInType(messages, np.float64, False)
        self.position_covariance = np.zeros((9, messages), dtype=np.float64)
        self.position_covariance_type = VecBuiltInType(messages, np.uint8, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.status.write_message(msg.status, t, count)
        self.latitude.write_message(msg.latitude, t, count)
        self.longitude.write_message(msg.longitude, t, count)
        self.position_covariance[:, count] = msg.position_covariance
        self.position_covariance_type.write_message(msg.position_covariance_type, t, count)

    def write_ros_msg(self, msg, msg_idx):
        self.header.write_ros_msg(msg.header, msg_idx)
        self.status.write_ros_msg(msg.status, msg_idx)
        self.latitude.write_ros_msg(msg.latitude, msg_idx)
        self.longitude.write_ros_msg(msg.longitude, msg_idx)
        msg.position_covariance = self.position_covariance[:, msg_idx]
        self.position_covariance_type.write_ros_msg(msg.position_covariance_type, msg_idx)

    def get_ros_msg(self, msg_idx):
        msg = sensor_msgs.msg.NavSatFix()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

    def get_latitude(self):
        return self.latitude.get_data()

    def get_longitude(self):
        return self.longitude.get_data()

class VecNavSatStatus(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.status = VecBuiltInType(messages, np.int8, False)
        self.service = VecBuiltInType(messages, np.uint16, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.status.write_message(msg.status, t, count)
        self.service.write_message(msg.service, t, count)

    def write_ros_msg(self, msg, msg_idx):
        self.status.write_ros_msg(msg.status, msg_idx)
        self.service.write_ros_msg(msg.service, msg_idx)

    def get_ros_msg(self, msg_idx):
        msg = sensor_msgs.msg.NavSatStatus()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

class VecPointField(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.name = []
        self.offset = VecBuiltInType(messages, np.uint32, False)
        self.datatype = VecBuiltInType(messages, np.uint8, False)
        self.count = VecBuiltInType(messages, np.uint32, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.name.append(msg.name)
        self.offset.write_message(msg.offset, t, count)
        self.datatype.write_message(msg.datatype, t, count)
        self.count.write_message(msg.count, t, count)

    def write_ros_msg(self, msg, msg_idx):
        msg.name = self.name[msg_idx]
        self.offset.write_ros_msg(msg.offset, msg_idx)
        self.datatype.write_ros_msg(msg.datatype, msg_idx)
        self.count.write_ros_msg(msg.count, msg_idx)

    def get_ros_msg(self, msg_idx):
        msg = sensor_msgs.msg.PointField()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

class VecRange(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.radiation_type = VecBuiltInType(messages, np.uint8, False)
        self.field_of_view = VecBuiltInType(messages, np.float32, False)
        self.min_range = VecBuiltInType(messages, np.float32, False)
        self.max_range = VecBuiltInType(messages, np.float32, False)
        self.range = VecBuiltInType(messages, np.float32, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.radiation_type.write_message(msg.radiation_type, t, count)
        self.field_of_view.write_message(msg.field_of_view, t, count)
        self.min_range.write_message(msg.min_range, t, count)
        self.max_range.write_message(msg.max_range, t, count)
        self.range.write_message(msg.range, t, count)

    def write_ros_msg(self, msg, msg_idx):
        self.header.write_ros_msg(msg.header, msg_idx)
        self.radiation_type.write_ros_msg(msg.radiation_type, msg_idx)
        self.field_of_view.write_ros_msg(msg.field_of_view, msg_idx)
        self.min_range.write_ros_msg(msg.min_range, msg_idx)
        self.max_range.write_ros_msg(msg.max_range, msg_idx)
        self.range.write_ros_msg(msg.range, msg_idx)

    def get_ros_msg(self, msg_idx):
        msg = sensor_msgs.msg.Range()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

class VecRegionOfInterest(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.x_offset = VecBuiltInType(messages, np.uint32, False)
        self.y_offset = VecBuiltInType(messages, np.uint32, False)
        self.height = VecBuiltInType(messages, np.uint32, False)
        self.width = VecBuiltInType(messages, np.uint32, False)
        self.do_rectify = VecBuiltInType(messages, np.uint8, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.x_offset.write_message(msg.x_offset, t, count)
        self.y_offset.write_message(msg.y_offset, t, count)
        self.height.write_message(msg.height, t, count)
        self.width.write_message(msg.width, t, count)
        self.do_rectify.write_message

    def write_ros_msg(self, msg, msg_idx):
        self.x_offset.write_ros_msg(msg.x_offset, msg_idx)
        self.y_offset.write_ros_msg(msg.y_offset, msg_idx)
        self.height.write_ros_msg(msg.height, msg_idx)
        self.width.write_ros_msg(msg.width, msg_idx)
        self.do_rectify.write_ros_msg(msg.do_rectify, msg_idx)

    def get_ros_msg(self, msg_idx):
        msg = sensor_msgs.msg.RegionOfInterest()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

class VecRelativeHumidity(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.relative_humidity = VecBuiltInType(messages, np.float64, False)
        self.variance = VecBuiltInType(messages, np.float64, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.relative_humidity.write_message(msg.relative_humidity, t, count)
        self.variance.write_message(msg.variance, t, count)

    def write_ros_msg(self, msg, msg_idx):
        self.header.write_ros_msg(msg.header, msg_idx)
        self.relative_humidity.write_ros_msg(msg.relative_humidity, msg_idx)
        self.variance.write_ros_msg(msg.variance, msg_idx)

    def get_ros_msg(self, msg_idx):
        msg = sensor_msgs.msg.RelativeHumidity()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

class VecTemperature(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.temperature = VecBuiltInType(messages, np.float64, False)
        self.variance = VecBuiltInType(messages, np.float64, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.temperature.write_message(msg.temperature, t, count)
        self.variance.write_message(msg.variance, t, count)

    def write_ros_msg(self, msg, msg_idx):
        self.header.write_ros_msg(msg.header, msg_idx)
        self.temperature.write_ros_msg(msg.temperature, msg_idx)
        self.variance.write_ros_msg(msg.variance, msg_idx)

    def get_ros_msg(self, msg_idx):
        msg = sensor_msgs.msg.Temperature()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]

    def get_temperature(self):
        return self.temperature

    def get_variance(self):
        return self.variance

class VecTimeReference(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.time_ref = VecTime(messages, False)
        self.source = []

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.time_ref.write_message(msg.time_ref, t, count)
        self.source.append(msg.source)

    def write_ros_msg(self, msg, msg_idx):
        self.header.write_ros_msg(msg.header, msg_idx)
        self.time_ref.write_ros_msg(msg.time_ref, msg_idx)
        msg.source = self.source[msg_idx]

    def get_ros_msg(self, msg_idx):
        msg = sensor_msgs.msg.TimeReference()
        self.write_ros_msg(msg, msg_idx)
        t = VecBagMsg.get_ros_msg(self, msg_idx)
        return [msg, t]
