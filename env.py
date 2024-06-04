#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
MIN_DISTANCE=0.5
ERROR_DISTANCE=0.1
class Env():
    def __init__(self, action_size):
        self.action_size = action_size
        self.position = ...
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

    def getState(self, camera):
        done = False
        pos_x,pos_y, dis_range = self.camera()
        min_ran = np.min(dis_range)
        posi_x=pos_x(np.argmin(dis_range))
        posi_y= pos_y(np.argmin(dis_range))
        state = [posi_x,posi_y,min_ran]
        if min_ran < ERROR_DISTANCE:
            done = True
        return state, done

    def setReward(self, state,done,action):
        done = False
        pre_action = action[-2]
        now_action = action[-1]
        min_ran = state[-1]
        if now_action == 0:#straight
            r_action = +0.2
        else:
            r_action = -0.1
        if (pre_action == 1 and now_action == 2) or (pre_action == 2 and now_action == 1):
            r_change = -0.3
        else:
            r_change = +0.2
        if min_ran < MIN_DISTANCE and min_ran > ERROR_DISTANCE:
            r_dis = -0.5
        else:
            r_dis = +0.5

        reward = r_change + r_action + r_dis

        return reward

    def step(self, action):
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done= self.getState(data)
        reward = self.setReward(state, action)

        return np.asarray(state), reward, done

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        self.goal_distance = self.getGoalDistace()
        state, done= self.getState(data)

        return np.asarray(state)