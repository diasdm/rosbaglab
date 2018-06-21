#Copyright 2018, David Dias
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np

from _bag_msg import VecBagMsg
from _std_msgs import *
from _std_msgs import *
from _geometry_msgs import *

class VecCustomMsg(VecBagMsg):
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

    def get_position_x(self):
        return self.pose.get_position_x()

    def get_position_y(self):
        return self.pose.get_position_y()

    def get_quat(self):
        return self.pose.get_quat()

    def get_time(self):
        return self.header.get_time()

class VecPwmValues(VecBagMsg):
    #Message File
    #Header header
    #float64[3] Force
    #float64[3] Moment
    #float64[6] Actuation
    #float64[6] PWM
    #float64[6] rpm
    #float64[3] Att_P_gains
    #float64[3] Att_I_gains
    #float64[3] Att_D_gains
    #float64[3] Current_rpy
    #float64[3] Desired_rpy
    #float64[3] Desired_position
    #float64[3] Desired_velocity
    #float64[3] Desired_omega
    #float64[3] Error_rpy
    #float64[3] err_int
    #float64 Time_Step
    #float64[3] Quat_error

    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.header = VecHeader(messages, False)
        self.force = VecBuiltInTypeArray(3, messages, np.float64, False)
        self.moment = VecBuiltInTypeArray(3, messages, np.float64, False)
        self.actuation = VecBuiltInTypeArray(6, messages, np.float64, False)
        self.pwm = VecBuiltInTypeArray(6, messages, np.float64, False)
        self.rpm = VecBuiltInTypeArray(6, messages, np.float64, False)
        self.att_p_gains = VecBuiltInTypeArray(3, messages, np.float64, False)
        self.att_i_gains = VecBuiltInTypeArray(3, messages, np.float64, False)
        self.att_d_gains = VecBuiltInTypeArray(3, messages, np.float64, False)
        self.current_rpy = VecBuiltInTypeArray(3, messages, np.float64, False)
        self.desired_rpy = VecBuiltInTypeArray(3, messages, np.float64, False)
        self.desired_position = VecBuiltInTypeArray(3, messages, np.float64, False)
        self.desired_velocity = VecBuiltInTypeArray(3, messages, np.float64, False)
        self.desired_omega = VecBuiltInTypeArray(3, messages, np.float64, False)
        self.error_rpy = VecBuiltInTypeArray(3, messages, np.float64, False)
        self.err_int = VecBuiltInTypeArray(3, messages, np.float64, False)
        self.time_Step = VecBuiltInType(messages, np.float64, False)
        self.quat_error = VecBuiltInTypeArray(3, messages, np.float64, False)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.header.write_message(msg.header, t, count)
        self.force.write_message(msg.Force, t, count)
        self.moment.write_message(msg.Moment, t, count)
        self.actuation.write_message(msg.Actuation, t, count)
        self.pwm.write_message(msg.PWM, t, count)
        self.rpm.write_message(msg.rpm, t, count)
        self.att_p_gains.write_message(msg.Att_P_gains, t, count)
        self.att_i_gains.write_message(msg.Att_I_gains, t, count)
        self.att_d_gains.write_message(msg.Att_D_gains, t, count)
        self.current_rpy.write_message(msg.Current_rpy, t, count)
        self.desired_rpy.write_message(msg.Desired_rpy, t, count)
        self.desired_position.write_message(msg.Desired_position, t, count)
        self.desired_velocity.write_message(msg.Desired_velocity, t, count)
        self.desired_omega.write_message(msg.Desired_omega, t, count)
        self.error_rpy.write_message(msg.Error_rpy, t, count)
        self.err_int.write_message(msg.err_int, t, count)
        self.time_Step.write_message(msg.Time_Step, t, count)
        self.quat_error.write_message(msg.Quat_error, t, count)

    def get_forces(self):
        return self.force

    def get_desired_rpy0(self):
        return self.desired_rpy.data[0]

    def get_desired_rpy1(self):
        return self.desired_rpy.data[1]

    def get_desired_rpy2(self):
        return self.desired_rpy.data[2]

    def get_current_rpy0(self):
        return self.current_rpy.data[0]

    def get_current_rpy1(self):
        return self.current_rpy.data[1]

    def get_current_rpy2(self):
        return self.current_rpy.data[2]

    def get_time(self):
        return self.header.get_time()
