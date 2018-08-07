#Copyright 2018, David Dias
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np
import matplotlib.pyplot as plt

plot_num = 0

def plot_position_xsmth_multiysmth(topic_list, topic_idx_list, title, xlabel, xfuncstr, ylabel, yfuncstr_l, axis_equal, legends_l=None):
    use_topic_legends = 0
    if legends_l == None:
        use_topic_legends = 1
    elif len(yfuncstr_l) != len(legends_l):
        print('The number of functions to plot must be the same as the number of legends!')
        return
    global plot_num
    fig = plt.figure('Figure ' + str(plot_num))

    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)

    for i in topic_idx_list:
        obj = topic_list[i].obj
        try:
            xfunc = getattr(obj, xfuncstr)
            x = xfunc()
            for idx, yfuncstr in enumerate(yfuncstr_l):
                yfunc = getattr(obj, yfuncstr)
                y = yfunc()
                if use_topic_legends:
                    plt.plot(x, y, label=topic_list[i].label)
                else:
                    plt.plot(x, y, label=legends_l[idx])
        except AttributeError:
            print('Can not plot '+title+' for '+topic_list[i].topic+', skipping')

    if axis_equal:
        plt.axis('equal')
    plt.legend()
    plt.show()
    plot_num += 1

def plot_position_x_y(topic_list, topic_idx_list):
    plot_position_xsmth_multiysmth(topic_list, topic_idx_list, 'X, Y position', 'X position [m]', 'get_position_x', 'Y position [m]', ['get_position_y'], True)

def plot_position_time_ysmth(topic_list, topic_idx_list, title, ylabel, yfuncstr_l):
    plot_position_xsmth_multiysmth(topic_list, topic_idx_list, title, 'Time [secs]', 'get_time', ylabel, yfuncstr_l, False)

def plot_position_time_multiysmth(topic_list, topic_idx_list, title, ylabel, yfunc, ylegends):
    plot_position_xsmth_multiysmth(topic_list, topic_idx_list, title, 'Time [secs]', 'get_time', ylabel, yfunc, False, ylegends)

def plot_position_time_x(topic_list, topic_idx_list):
    plot_position_time_ysmth(topic_list, topic_idx_list, 'X position over time', 'X position [m]', ['get_position_x'])

def plot_position_time_y(topic_list, topic_idx_list):
    plot_position_time_ysmth(topic_list, topic_idx_list, 'Y position over time', 'Y position [m]', ['get_position_y'])

def plot_position_time_z(topic_list, topic_idx_list):
    plot_position_time_ysmth(topic_list, topic_idx_list, 'Z position over time', 'Z position [m]', ['get_position_z'])

def plot_orientation_roll(topic_list, topic_idx_list):
    plot_position_time_ysmth(topic_list, topic_idx_list, 'Roll rotation over time', 'Roll rotation [deg]', ['get_roll'])

def plot_orientation_pitch(topic_list, topic_idx_list):
    plot_position_time_ysmth(topic_list, topic_idx_list, 'Pitch rotation over time', 'Pitch rotation [deg]', ['get_pitch'])

def plot_orientation_yaw(topic_list, topic_idx_list):
    plot_position_time_ysmth(topic_list, topic_idx_list, 'Yaw rotation over time', 'Yaw rotation [deg]', ['get_yaw'])

def plot_orientation(topic_list, topic_idx_list):
    plot_orientation_roll(topic_list, topic_idx_list)
    plot_orientation_pitch(topic_list, topic_idx_list)
    plot_orientation_yaw(topic_list, topic_idx_list)

def plot_linear_acceleration(topic_list, topic_idx_list):
    plot_position_time_ysmth(topic_list, topic_idx_list, 'X linear accelerarion over time', 'X linear accelerarion [m/s^2]', ['get_linear_acceleration_x'])
    plot_position_time_ysmth(topic_list, topic_idx_list, 'Y linear accelerarion over time', 'Y linear accelerarion [m/s^2]', ['get_linear_acceleration_y'])
    plot_position_time_ysmth(topic_list, topic_idx_list, 'Z linear accelerarion over time', 'Z linear accelerarion [m/s^2]', ['get_linear_acceleration_z'])

def plot_angular_velocity_roll(topic_list, topic_idx_list):
    plot_position_time_ysmth(topic_list, topic_idx_list, 'X angular velocity over time', 'X angular velocity [rad/s^2]', ['get_angular_velocity_x'])

def plot_angular_velocity_pitch(topic_list, topic_idx_list):
    plot_position_time_ysmth(topic_list, topic_idx_list, 'Y angular velocity over time', 'Y angular velocity [rad/s^2]', ['get_angular_velocity_y'])

def plot_angular_velocity_yaw(topic_list, topic_idx_list):
    plot_position_time_ysmth(topic_list, topic_idx_list, 'Z angular velocity over time', 'Z angular velocity [rad/s^2]', ['get_angular_velocity_z'])

def plot_angular_velocity(topic_list, topic_idx_list):
    plot_angular_velocity_roll(topic_list, topic_idx_list)
    plot_angular_velocity_pitch(topic_list, topic_idx_list)
    plot_angular_velocity_yaw(topic_list, topic_idx_list)

def plot_position(topic_list, topic_idx_list):
    plot_position_x_y(topic_list, topic_idx_list)
    plot_position_time_x(topic_list, topic_idx_list)
    plot_position_time_y(topic_list, topic_idx_list)

def plot_odometry(topic_list, topic_idx_list):
    plot_position(topic_list, topic_idx_list)
    plot_orientation(topic_list, topic_idx_list)

def plot_imu(topic_list, topic_idx_list):
    plot_orientation(topic_list, topic_idx_list)
    plot_linear_acceleration(topic_list, topic_idx_list)
    plot_angular_velocity(topic_list, topic_idx_list)

def plot_latitude_time(topic_list, topic_idx_list):
    plot_position_time_ysmth(topic_list, topic_idx_list, 'Latitude over time', 'Latitude [deg]', ['get_latitude'])

def plot_longitude_time(topic_list, topic_idx_list):
    plot_position_time_ysmth(topic_list, topic_idx_list, 'Longitude over time', 'Longitude [deg]', ['get_longitude'])

def plot_gps_coordinates(topic_list, topic_idx_list):
    plot_position_xsmth_multiysmth(topic_list, topic_idx_list, 'GPS coordinates', 'Latitude [deg]', 'get_latitude', 'Longitude [deg]', ['get_longitude'], True)

def plot_topics_list(topics_list):
    plot_position(topics_list, range(len(topics_list)))
    plot_orientation(topics_list, range(len(topics_list)))
    plot_linear_acceleration(topics_list, range(len(topics_list)))
    plot_angular_velocity(topics_list, range(len(topics_list)))

def plot_by_type(topic_list, topic_idx_list):
    type = topic_list[topic_idx_list[0]].type

    if type == 'nav_msgs/Odometry':
        plot_odometry(topic_list, topic_idx_list)
    elif type == 'sensor_msgs/Imu':
        plot_imu(topic_list, topic_idx_list)
    else:
        print('ERROR, no plot method for ' + type)

def plot_same_type(topic_list, bagprop):
    for topic_idx_list in bagprop.diff_topic_idx:
        plot_by_type(topic_list, topic_idx_list)
