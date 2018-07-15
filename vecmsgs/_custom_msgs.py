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
