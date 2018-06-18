#Copyright 2018, David Dias
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np
from tf.transformations import * 

def quat2roll(quat):
    t = np.arctan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[0]**2 + quat[1]**2))
    #t = np.empty(quat.shape[1], dtype=np.float64)
    #for i in range(quat.shape[1]):
        #t[i] = euler_from_quaternion(quat[:,i])[0]
    return t

def quat2pitch(quat):
    t = np.arcsin(2.0 * (quat[3] * quat[1] - quat[2] * quat[0]))
    #t = np.empty(quat.shape[1], dtype=np.float64)
    #for i in range(quat.shape[1]):
        #t[i] = euler_from_quaternion(quat[:,i])[1]
    return t

def quat2yaw(quat):
    t = np.arctan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1]**2 + quat[2]**2))
    #t = np.empty(quat.shape[1], dtype=np.float64)
    #for i in range(quat.shape[1]):
        #t[i] = euler_from_quaternion(quat[:,i])[2]
    return t

def quat2eul(q):
    return np.array([quat2roll(q), quat2pitch(q), quat2yaw(q)])
