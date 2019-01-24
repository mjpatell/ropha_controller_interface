#!/usr/bin/python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal

from trajectory_msgs.msg import JointTrajectoryPoint

import control_msgs.msg
from geometry_msgs.msg import *
import roslib
import json
import argparse
import numpy
import math
from sensor_msgs.msg import JointState
import threading
import thread
import sys
import time
import os
import rospkg
import datetime

class Trajectory:
    def __init__(self, arm_name):
        """
        Initializing the Parameters and the Client definition to send to the controller
        """
        self.goal = FollowJointTrajectoryGoal()
        self.client = actionlib.SimpleActionClient(arm_name+'/joint_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        self.client.wait_for_server(rospy.Duration(5))

    def add_point(self, positions, velocity, acceleration, time):
        """
        :param positions: adds all the point positions available in the given file
        :param velocity: adds the given velocity at which the Robotarm should move
        :param acceleration: adds the given acceleration value at which the Robotarm should move
        :param time: Time, which the Robotarm take to reach to the next point (Time_from_start)
        """
        point = JointTrajectoryPoint()
        point.positions = positions
        print(positions)
        point.velocities = velocity
        print(velocity)
        point.accelerations = acceleration
        print(acceleration)
        point.time_from_start = rospy.Duration(time)
        print(time)
        self.goal.trajectory.points.append(point)

    def start(self):
        """
        First step of the Trajectory Goal, which sends all the details of Goal
        (Trajectory Points, Velocity, Acceleration) to the Controller
        """
        self.goal.trajectory.header.stamp = rospy.Time(0.0)
        self.client.send_goal(self.goal)

    def stop(self):
        """
        can be used to stop the Trajectory in case of some Collision, Emergency or invalid point details
        """
        self.client.cancel_goal()

    def clear_goal(self):
        self.goal.trajectory.points.pop(len(self.goal.trajectory.points)-1)

    def wait(self, timeout):
        """
        waits for the controller to respond, if it has got all the required details and if it has started executing the
        result in a given Timeout
        :param timeout: Time, within which the Controller should respond and give the result, if not, it should cancel
        the Goal
        """
        self.client.wait_for_result(timeout=rospy.Duration(timeout))


    def goal_defination(self):
        timeout = 10.0

        for i in range(len(pos)):

            self.add_point(pos[i], vel[i], acc[i], time[i])
            #print(pos[i])
            #print(vel[i])
            #print(acc[i])
            #print(time[i])
            #print(joint)
            print(i)
        self.start()

        #time_before_result = self.client.wait_for_result(rospy.Duration(10))

        #if (time_before_result and self.client.get_state() is 10):
        #    print "Finish first trajectory execution"

        self.clear_goal()
        self.client.wait_for_result()

    def read_data(self):
        joint_names = []; position_values = []; time = []; velocity = []; acceleration = []
        for item in data[0]['points_'][0]['state_']['joints_']:
            joint_names.append(str(item['name_']))
        #print(joint_names)

        for i in range(len(data[0]['points_'])):
            time_i = data[0]['points_'][i]['time_']
            time.append(time_i)
            temp_value = []
            for item1 in data[0]['points_'][i]['state_']['joints_']:
                temp_value.append(item1['value_'])
            position_values.append(temp_value)

            temp_vel = []
            temp_acc = []
            for ii in data[0]['points_'][i]['state_']['joints_']:
                temp_vel.append(ii['velocity_'])
                temp_acc.append(ii['acceleration_'])
            velocity.append(temp_vel)
            acceleration.append(temp_acc)

        return [joint_names, position_values, time, velocity, acceleration]

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input_arm_name', default='arm_left', help='enter the arm name')
    parser.add_argument('input_gripper_name', default='gripper_left', help='enter the gripper name')
    args, unknown = parser.parse_known_args()
    args = parser.parse_args()

    rospy.loginfo("Initializing the Node")
    rospy.init_node("follow_joint_trajectory_goal")
    #defining the Node
    rospy.loginfo("Running, Ctrl+C to quit")
    rospy.loginfo('Follow Joint Trajectory Motion Planning')
    trajectory = Trajectory(args.input_arm_name)
    #trajectory = Trajectory(args.input_gripper_name)
    #defining the object for the class to use the functions
    rospy.loginfo("Trajectory defined")
    rospy.loginfo("Parameters defined")

    with open(os.path.join(rospack.get_path("ropha_controller_interface"), "src", "trajectory.json")) as f:
        data = json.load(f)
        [joint, pos, time, vel, acc] = trajectory.read_data()
        #print(joint)
        trajectory.goal.trajectory.joint_names = joint
        #print(trajectory.goal.trajectory.joint_names)
        trajectory.goal_defination()

    rospy.loginfo("Goal sent")
    rospy.loginfo('Follow Joint Trajectory Successfully Completed...')
