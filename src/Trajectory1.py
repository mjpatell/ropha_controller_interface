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
        point.velocities = velocity
        point.accelerations = acceleration
        point.time_from_start = rospy.Duration(time)

        print "time_from_start: ", point.time_from_start

        self.goal.trajectory.points.append(point)

    def clear_goal(self):
        self.goal.trajectory.points.pop(len(self.goal.trajectory.points)-1)

    def start(self):
        """
        First step of the Trajectory Goal, which sends all the details of Goal
        (Trajectory Points, Velocity, Acceleration) to the Controller
        """
        self.goal.trajectory.header.stamp = rospy.Time.now()
        self.client.send_goal(self.goal)

    def stop(self):
        """
        can be used to stop the Trajectory in case of some Collision, Emergency or invalid point details
        """
        self.client.cancel_goal()

    def wait(self, timeout):
        """
        waits for the controller to respond, if it has got all the required details and if it has started executing the
        result in a given Timeout
        :param timeout: Time, within which the Controller should respond and give the result, if not, it should cancel
        the Goal
        """
        self.client.wait_for_result(timeout=rospy.Duration(timeout))

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

    def read_data(self):
    #reads all the data from a given file to parse it
        rospack = rospkg.RosPack()
        with open(os.path.join(rospack.get_path("ropha_controller_interface"), "src", "Trajectory_Data_Left_arm.json")) as f:
            data = json.load(f)
            joint_names = []
            position_values = []
            for item2 in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:
                joint_names.append(str(item2['name_']))
            print(joint_names)
            #read joint names specified in file
            self.goal.trajectory.joint_names.append(str(joint_names))
            print(joint_names)
            #adding the joint names to Trajectory message
            #joint_name.append(item['name_'])


            for i in range(len(data['Template']['motions_'][0]['references_']['states_'])):
                #read the position trajectory from the file
                c = []
                for item in data['Template']['motions_'][0]['references_']['states_'][i]['joints_']:
                    c.append(item['value_'])
                position_values.append(c)

            vel = data['Template']['motions_'][0]['properties_']['dynamics_']['velocities_']
            #reading the given velocity value from the file
            acc = data['Template']['motions_'][0]['properties_']['dynamics_']['accelerations_']
            #reading the acceleration value from the file

            return [joint_names, position_values, vel, acc]
            #returns all the above defined values for the future usage


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input_arm_name', default='arm_left', help='enter the arm name')
    args, unknown = parser.parse_known_args()
    args = parser.parse_args()

    rospy.loginfo("Initializing the Node")
    rospy.init_node("follow_joint_trajectory_goal")
    #defining the Node
    rospy.loginfo("Running, Ctrl+C to quit")
    rospy.loginfo('Follow Joint Trajectory Motion Planning')
    trajectory = Trajectory(args.input_arm_name)
    #defining the object for the class to use the functions
    rospy.loginfo("Trajectory defined")
    rospy.loginfo("Parameters defined")
    [joints_name, joints_position, joints_velocity, joints_acceleration] = trajectory.read_data()


    print ("read: joint names: ", joints_name)
    print ("read: joint position: ", joints_position)
    print ("read: joint velocity: ", joints_velocity)
    print ("read: joint acceleration: ", joints_acceleration)

    rospy.loginfo("Data Reading finished")

    traj_time = 0
    timeout = 3.0

    try:
        current_pose = rospy.wait_for_message("/" + args.input_arm_name + "/joint_states", JointState, timeout=timeout).position
        trajectory.goal.trajectory.joint_names = rospy.wait_for_message("/" + args.input_arm_name + "/joint_states", JointState, timeout=timeout).name
        # print ("Current pose: ", current_pose)
    except rospy.ROSException as e:
        rospy.logwarn("no joint states received from %s within timeout of %ssec. using default point time of 8sec.",
                      "arm", str(timeout))

    print ("current joint states: ", current_pose)

    '''
    for i in range(len(joints_position)):
        point_time = trajectory.calculate_point_time(current_pose, joints_position[i], joints_velocity[i],
                                                     joints_acceleration[i])
        #pushing the point data for all the trajectory points
        current_pose = joints_position[i]
        trajectory.add_point(joints_position[i], joints_velocity, joints_acceleration, (timeout+traj_time))
        trajectory.goal.trajectory.joint_names = joints_name[i]
        traj_time += point_time
    '''

    for point in range(len(joints_position)):
        point_time = trajectory.calculate_point_time(current_pose, joints_position[point], joints_velocity[point],
                                                         joints_acceleration[point])
        print "point time: ", point_time
        #pushing the point data for all the trajectory points
        #current_pose = joints_position[0]
        print joints_name[point]
        trajectory.add_point(joints_position[point], joints_velocity, joints_acceleration, timeout)
        traj_time += point_time

        print "traj_time: ", traj_time

        trajectory.client.send_goal(trajectory.goal)
        time_before_result = trajectory.client.wait_for_result(rospy.Duration(10))

        if (time_before_result and trajectory.client.get_state() is 3):
            print "Finish first trajectory execution"

        trajectory.clear_goal()

    rospy.spin()
    #trajectory.start()
    #send the goal to the client for the execution
    rospy.loginfo("Goal sent")
    trajectory.client.wait_for_result()

    rospack = rospkg.RosPack()
    file_path = rospack.get_path("ropha_controller_interface")
    time=datetime.datetime.now()

    json_data = {}
    #json_data['Tamplate'] = 'Joint_Names', 'Positions', 'Velocity', 'Time_from_Start', 'Effort', "Joint_Positions"
    json_data['Joint_Names'] = []
    for i in range(len(trajectory.goal.trajectory.joint_names)):
name = trajectory.goal.trajectory.joint_names[i]
        json_data['Joint_Names'].append(name)
    json_data['Points'] = []
    for i in range(len(trajectory.goal.trajectory.points)):
        point = str(trajectory.goal.trajectory.points[i])
        json_data['Points'].append(point)
    #for i in range(len(trajectory.goal.trajectory.joint_names)):
    #    joint_data.append(trajectory.goal.trajectory.joint_names(i))
    #    json_data['Tamplate']['Joint_Names'] = joint_data
    with open(file_path+'/Trajectory_Goal_Written'+str(time)+'.json', 'w') as f:
        json.dump(json_data, f)

    rospy.loginfo('Follow Joint Trajectory Successfully Completed...')
