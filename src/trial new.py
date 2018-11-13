#!/usr/bin/python

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

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
import yaml
import rospkg



class Trajectory:
    def __init__(self):                 #initializing all the parameters
        self.goal = FollowJointTrajectoryGoal()
        self.client = actionlib.SimpleActionClient('/arm_left/joint_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        self.client.wait_for_server(rospy.Duration(5))

    def add_point(self, positions, velocity, acceleration, time):       #adds the trajectory points
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocity
        point.accelerations = acceleration
        point.time_from_start = rospy.Duration(time)
        self.goal.trajectory.points.append(point)
        #print(self.goal.trajectory.points)
        #return self.point, self.goal, positions, time

    def start(self):
        self.goal.trajectory.header.stamp = rospy.Time.now()
        self.client.send_goal(self.goal)
        self.client.wait_for_result()

    def stop(self):
        self.client.cancel_goal()

    def wait(self, timeout):
        self.client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self.client.get_result()

    def msg2json(self, msg):
        y = yaml.load(str(msg))
        return json.dumps(y, indent=4)

    def calculate_point_time(self, start_pos, end_pos, default_vel, default_acc):
        try:
            d_max = max(list(abs(numpy.array(start_pos) - numpy.array(end_pos))))
            t1 = default_vel / default_acc
            s1 = default_acc / 2 * t1 ** 2
            if (2 * s1 < d_max):
                # with constant velocity phase (acc, const vel, dec)
                # 1st phase: accelerate from v=0 to v=default_vel with a=default_acc in t=t1
                # 2nd phase: constante velocity with v=default_vel and t=t2
                # 3rd phase: decceleration (analog to 1st phase)
                s2 = d_max - 2 * s1
                t2 = s2 / default_vel
                t = 2 * t1 + t2
            else:
                # without constant velocity phase (only acc and dec)
                # 1st phase: accelerate from v=0 to v=default_vel with a=default_acc in t=t1
                # 2nd phase: missing because distance is to short (we already reached the distance with the acc and dec phase)
                # 3rd phase: decceleration (analog to 1st phase)
                t = math.sqrt(d_max / default_acc)
            point_time = max(t, 0.4)  # use minimal point_time
        except ValueError as e:
            print ("Value Error", e)
            print ("Likely due to mimic joints. Using default point_time: 3.0 [sec]")
            point_time = 3.0  # use default point_time
        return point_time

    def write_data(self, joint_name, point_positions, velocity, acceleration):              #writes all the read data in a file
        with open('shyam.txt', 'w') as f:
            json.dump("Joint_Names:", f)
            json.dump(joint_name, f)
            f.write('\n \n')
            f.write("Position Data for the Points:")
            f.write('\n')
            json.dump(point_positions, f)
            f.write('\n\n')
            json.dump("Velocity:", f)
            json.dump(velocity, f)
            f.write('\n\n')
            json.dump("Acceleration:", f)
            json.dump(acceleration, f)
            f.write('\n\n')

    def read_data(self):                #reads all the data from a given file to parse it
        print("Read Data from File")
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('ropha_ws'), 'config', 'shyam_json_1.json')) as f:
            data = json.load(f)
            joint_name = []; position_values = []
            for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:        #read joint names specified in file
                self.goal.trajectory.joint_names.append(str(item['name_']))      #adding the joint names to Trajectory message
                joint_name.append(item['name_'])
            print(joint_name)

            for i in range(len(data['Template']['motions_'][0]['references_']['states_'])):  #read the position trajectory from the file
                c = []
                for item in data['Template']['motions_'][0]['references_']['states_'][i]['joints_']:
                    c.append(item['value_'])
                position_values.append(c)

            d = []; e = []
            vel = data['Template']['motions_'][0]['properties_']['dynamics_']['velocities_']        #reading the given velocity value from the file
            acc = data['Template']['motions_'][0]['properties_']['dynamics_']['accelerations_']     #reading the acceleration value from the file
            d.append(vel)
            e.append(acc)
            self.write_data(joint_name, position_values, vel, acc)          #passing all the values to write them in a text file for a simple understanding
            print(position_values)
            print(joint_name)
            print(vel)
            print(acc)
            print("Read Data Finish")

            return [joint_name, position_values, vel, acc]         #returns all the above defined values for the future usage


if __name__ == '__main__':
    #parser = argparse.ArgumentParser()
    #parser.add_argument('--base_link', required=True, default='base_link', help='base link of the kinematic chain')

    #args, unknown = parser.parse_known_args()

    print("Initializing the Node")
    rospy.init_node("follow_joint_trajectory_goal")         #defining the Node
    print("Running, Ctrl+C to quit")
    rospy.loginfo('Follow Joint Trajectory Motion Planning')
    trajectory = Trajectory()                               #defining the object for the class to use the functions
    print("Trajectory defined")
    trajectory.__init__()
    print("Parameters defined")
    [joi,pos,vel,acc] = trajectory.read_data()
    print("Data Reading finished")
    traj_time = 0
    timeout = 3.0

    try:
        current_pose = rospy.wait_for_message("/" + "arm_left" + "/joint_states", JointState, timeout=timeout).position
        # print ("Current pose: ", current_pose)
    except rospy.ROSException as e:
        rospy.logwarn("no joint states received from %s within timeout of %ssec. using default point time of 8sec.",
                      "arm", str(timeout))
    for i in range(len(pos)):
        point_time = trajectory.calculate_point_time(current_pose, pos[i], vel[i], acc[i])       #pushing the point data for all the trajectory points
        current_pose = pos[i]
        trajectory.add_point(pos[i], vel, acc, (point_time+traj_time))
        traj_time += point_time
    trajectory.start()          #send the goal to the client for the execution
    print("Goal sent")
    print(trajectory.goal.trajectory.points)
    #trajectory.wait()
    print("Follow Joint Trajectory Successfully Completed...")
    rospy.spin()
