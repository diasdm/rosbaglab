#Copyright 2018, David Dias
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np

import _std_msgs
import _geometry_msgs
import _nav_msgs
import _sensor_msgs
import _custom_msgs

def get_message_obj(type, messages):
        return {
            # _std_msgs http://wiki.ros.org/std_msgs
            'std_msgs/Bool': _std_msgs.VecBuiltInType(messages, np.bool_),
            'std_msgs/Byte': _std_msgs.VecBuiltInType(messages, np.uint8),
            'std_msgs/Char': _std_msgs.VecBuiltInType(messages, np.uint8),
            'std_msgs/ColorRGBA': _std_msgs.VecColorRGBA(messages),
            'std_msgs/Duration': _std_msgs.VecDuration(messages),
            'std_msgs/Empty': _std_msgs.VecEmpty(messages),
            'std_msgs/Float32': _std_msgs.VecBuiltInType(messages, np.float32),
            'std_msgs/Float64': _std_msgs.VecBuiltInType(messages, np.float64),
            'std_msgs/Header': _std_msgs.VecHeader(messages),
            'std_msgs/Int16': _std_msgs.VecBuiltInType(messages, np.int16),
            'std_msgs/Int32': _std_msgs.VecBuiltInType(messages, np.int32),
            'std_msgs/Int64': _std_msgs.VecBuiltInType(messages, np.int64),
            'std_msgs/Int8': _std_msgs.VecBuiltInType(messages, np.int8),
            'std_msgs/String': _std_msgs.VecString(messages),
            'std_msgs/Time': _std_msgs.VecTime(messages),
            'std_msgs/UInt16': _std_msgs.VecBuiltInType(messages, np.uint16),
            'std_msgs/UInt32': _std_msgs.VecBuiltInType(messages, np.uint32),
            'std_msgs/UInt64': _std_msgs.VecBuiltInType(messages, np.uint64),
            'std_msgs/UInt8': _std_msgs.VecBuiltInType(messages, np.uint8),
            # _geometry_msgs http://wiki.ros.org/geometry_msgs
            'geometry_msgs/Accel': _geometry_msgs.VecAccel(messages),
            'geometry_msgs/AccelStamped': _geometry_msgs.VecAccelStamped(messages),
            'geometry_msgs/AccelWithCovariance': _geometry_msgs.VecAccelWithCovariance(messages),
            'geometry_msgs/AccelWithCovarianceStamped': _geometry_msgs.VecAccelWithCovarianceStamped(messages),
            'geometry_msgs/Inertia': _geometry_msgs.VecInertia(messages),
            'geometry_msgs/InertiaStamped': _geometry_msgs.VecInertiaStamped(messages),
            'geometry_msgs/Point': _geometry_msgs.VecPoint(messages),
            'geometry_msgs/Point32': _geometry_msgs.VecPoint32(messages),
            'geometry_msgs/PointStamped': _geometry_msgs.VecPoint(messages),
            'geometry_msgs/Pose': _geometry_msgs.VecPose(messages),
            'geometry_msgs/Pose2D': _geometry_msgs.VecPose(messages),
            'geometry_msgs/PoseStamped': _geometry_msgs.VecPoseStamped(messages),
            'geometry_msgs/PoseWithCovariance': _geometry_msgs.VecPoseWithCovariance(messages),
            'geometry_msgs/PoseWithCovarianceStamped': _geometry_msgs.VecPoseWithCovarianceStamped(messages),
            'geometry_msgs/Quaternion': _geometry_msgs.VecQuaternion(messages),
            'geometry_msgs/QuaternionStamped': _geometry_msgs.VecQuaternionStamped(messages),
            'geometry_msgs/Transform': _geometry_msgs.VecTransform(messages),
            'geometry_msgs/TransformStamped': _geometry_msgs.VecTransform(messages),
            'geometry_msgs/Twist': _geometry_msgs.VecTwist(messages),
            'geometry_msgs/TwistStamped': _geometry_msgs.VecTwistStamped(messages),
            'geometry_msgs/TwistWithCovariance': _geometry_msgs.VecTwistWithCovariance(messages),
            'geometry_msgs/TwistWithCovarianceStamped': _geometry_msgs.VecTwistWithCovarianceStamped(messages),
            'geometry_msgs/Vector3': _geometry_msgs.VecVector3(messages),
            'geometry_msgs/Vector3Stamped': _geometry_msgs.VecVector3Stamped(messages),
            'geometry_msgs/Wrench': _geometry_msgs.VecWrench(messages),
            'geometry_msgs/WrenchStamped': _geometry_msgs.VecWrenchStamped(messages),
            # _nav_msgs http://wiki.ros.org/nav_msgs
            'nav_msgs/MapMetaData': _nav_msgs.VecMapMetaData(messages),
            'nav_msgs/Odometry': _nav_msgs.VecOdometry(messages),
            # _sensor_msgs http://wiki.ros.org/sensor_msgs
            'sensor_msgs/FluidPressure': _sensor_msgs.VecFluidPressure(messages),
            'sensor_msgs/Illuminance': _sensor_msgs.VecIlluminance(messages),
            'sensor_msgs/Imu': _sensor_msgs.VecImu(messages),
            'sensor_msgs/JoyFeedback': _sensor_msgs.VecJoyFeedback(messages),
            'sensor_msgs/MagneticField': _sensor_msgs.VecMagneticField(messages),
            'sensor_msgs/NavSatFix': _sensor_msgs.VecNavSatFix(messages),
            'sensor_msgs/NavSatStatus': _sensor_msgs.VecNavSatStatus(messages),
            'sensor_msgs/PointField': _sensor_msgs.VecPointField(messages),
            'sensor_msgs/Range': _sensor_msgs.VecRange(messages),
            'sensor_msgs/RegionOfInterest': _sensor_msgs.VecRegionOfInterest(messages),
            'sensor_msgs/RelativeHumidity': _sensor_msgs.VecRelativeHumidity(messages),
            'sensor_msgs/Temperature': _sensor_msgs.VecTemperature(messages),
            'sensor_msgs/TimeReference': _sensor_msgs.VecTimeReference(messages),
            # _custom_msgs
            'some_msgs/CustomMsgs': _custom_msgs.VecCustomMsg(messages),
            }.get(type, None)
