#Copyright 2018, David Dias
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np
from collections import namedtuple
from rospy.rostime import Time

class VecBagTime:
    def __init__(self, messages):
        self.secs = np.zeros(messages, dtype=np.uint32)
        self.nsecs = np.zeros(messages, dtype=np.uint32)
        self.ft = np.zeros(messages, dtype=np.float64)

class VecBagMsg:
    def __init__(self, messages = 0):
        self.messages = messages
        self.bagtime = VecBagTime(messages)
        
    def write_message(self, t, count):
        if self.messages != 0: 
            self.bagtime.secs[count] = t.secs
            self.bagtime.nsecs[count] = t.nsecs
            self.bagtime.ft[count] = np.float64(t.secs + np.float64(t.nsecs) * 10**(-9))
    
    def get_time(self):
        return self.bagtime.ft
    
    def set_bag_time(self, ft):
        self.bagtime.secs = np.array(np.trunc(ft), dtype=np.uint32)
        self.bagtime.nsecs = np.array((ft - self.bagtime.secs) * 10**9, np.uint32)
        self.bagtime.ft = ft
    
    def get_ros_msg(self, msg_idx):
        t = Time()
        t.secs = int(self.bagtime.secs[msg_idx])
        t.nsecs = int(self.bagtime.nsecs[msg_idx])
        return t
