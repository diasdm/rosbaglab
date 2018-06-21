#Copyright 2018, David Dias
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np

from _bag_msg import *

class VecBuiltInType(VecBagMsg):
    def __init__(self, messages, numpy_type, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.data = np.zeros(messages, dtype=numpy_type)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.data[count] = msg

class VecBuiltInTypeArray(VecBagMsg):
    def __init__(self, array_len, messages, numpy_type, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.array_len = array_len
        self.data = np.ndarray(shape=(array_len, messages), dtype=numpy_type)

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.data[:,count] = msg

class VecColorRGBA(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.r = np.zeros(messages, dtype=np.float32)
        self.g = np.zeros(messages, dtype=np.float32)
        self.b = np.zeros(messages, dtype=np.float32)
        self.a = np.zeros(messages, dtype=np.float32)
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.r = msg.r
        self.g = msg.g
        self.b = msg.b
        self.a = msg.a

class VecDuration(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.secs = np.zeros(messages, dtype=np.int32)
        self.nsecs = np.zeros(messages, dtype=np.int32)
        # time as float
        self.ft = np.zeros(messages, dtype=np.float64())
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.secs[count] = msg.secs
        self.nsecs[count] = msg.nsecs
        self.ft[count] = np.float64(msg.secs + np.float64(msg.nsecs) * 10**(-9))

class VecEmpty(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        
    def write_message(self, msg, t, count):
        return

class VecHeader(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.seq = np.zeros(messages, dtype=np.uint32)
        self.stamp = VecTime(messages, False)
        self.frame_id = ''

    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.seq[count] = msg.seq
        self.stamp.write_message(msg.stamp, t, count)
        self.frame_id = msg.frame_id
            
    def get_time(self):
        return self.stamp.get_time()
    
    def set_time(self, ft):
        return self.stamp.set_time(ft)
    
    def write_ros_msg(self, msg, msg_idx):
        self.stamp.write_ros_msg(msg.stamp, msg_idx)
        msg.frame_id = self.frame_id

class VecString(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.data = []
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.data.append(msg)

class VecTime(VecBagMsg):
    def __init__(self, messages, bagmsg_length = True):
        if bagmsg_length:
            VecBagMsg.__init__(self, messages)
        else:
            VecBagMsg.__init__(self, 0)
        self.secs = np.zeros(messages, dtype=np.uint32)
        self.nsecs = np.zeros(messages, dtype=np.uint32)
        # time as float
        self.ft = np.zeros(messages, dtype=np.float64())
        
    def write_message(self, msg, t, count):
        VecBagMsg.write_message(self, t, count)
        self.secs[count] = msg.secs
        self.nsecs[count] = msg.nsecs
        self.ft[count] = np.float64(msg.secs + np.float64(msg.nsecs) * 10**(-9))
    
    def get_time(self):
        return self.ft
    
    def set_time(self, ft):
        self.secs = np.array(np.trunc(ft), dtype=np.uint32)
        self.nsecs = np.array((ft - self.secs) * 10**9, np.uint32)
        self.ft = ft
    
    def write_ros_msg(self, msg, msg_idx):
        msg.secs = int(self.secs[msg_idx])
        msg.nsecs = int(self.nsecs[msg_idx])
