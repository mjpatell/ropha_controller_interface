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


class Trajectory:
    def __init__(self):                 #initializing all the parameters
        self.object1 = FollowJointTrajectoryGoal()
        self.position = PoseStamped()
        self.point = JointTrajectoryPoint()
        self.goal_time_tolerance = rospy.Duration()
        self.object1.goal_time_tolerance = self.goal_time_tolerance
        self.client = actionlib.SimpleActionClient('/arm_left/joint_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        self.client.wait_for_server(rospy.Duration(5))

    def create_frame(self, stamp_secs = 0):
        self.object1.trajectory.header.stamp_secs = rospy.Time(0)
        return self.object1

    def add_point(self, positions, velocity, acceleration, time):       #adds the trajectory points
        self.point.positions = positions
        self.point.velocities = velocity
        self.point.accelerations = acceleration
        self.point.time_from_start = rospy.Duration(time)
        self.object1.trajectory.points.append(self.point)
        return self.point, self.object1, positions, time

    def start(self):
        self.object1.trajectory.header.stamp = rospy.Time(0)
        print("position data", self.object1.trajectory.points[0])
        self.client.send_goal(self.object1)

    def stop(self):
        self.client.cancel_goal()

    def wait(self, timeout):
        self.client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self.client.get_result()

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
        with open('/home/myp-stud1/ropha_ws/src/ropha_controller_interface/src/shyam_json_1.json') as f:
            data = json.load(f)
            joint_name = []; position_values = []
            for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:        #read joint names specified in file
                self.object1.trajectory.joint_names.append(str(item['name_']))      #adding the joint names to Trajectory message

            for i in range(len(data['Template']['motions_'][0]['references_']['states_'])):         #read the position trajectory from the file
                c = []
                for item in data['Template']['motions_'][0]['references_']['states_'][i]['joints_']:
                    c.append(item['value_'])
                position_values.append(c)
                #self.point.positions.append(position_values)        #adding the trajectory point data to the trajectory message

            d = []; e = []
            vel = data['Template']['motions_'][0]['properties_']['dynamics_']['velocities_']        #reading the given velocity value from the file
            acc = data['Template']['motions_'][0]['properties_']['dynamics_']['accelerations_']     #reading the acceleration value from the file
            d.append(vel)
            e.append(acc)
            self.write_data(joint_name, position_values, vel, acc)          #passing all the values to write them in a text file for a simple understanding
            self.point.velocities.append(vel)           #adding the velocity value to the trajectory message
            self.point.accelerations.append(acc)        #adding the acceleration value to the trajectory message
            print(position_values)
            print(joint_name)
            print(vel)
            print(acc)
            print("Read Data Finish")

            return [joint_name, position_values, vel, acc, 0.0]         #returns all the above defined values for the future usage


if __name__ == '__main__':
    #parser = argparse.ArgumentParser()
    #parser.add_argument('--base_link', required=True, default='base_link', help='base link of the kinematic chain')

    #args, unknown = parser.parse_known_args()

    print("Initializing the Node")
    rospy.init_node("follow_joint_trajectory_goal")         #defining the Node
    print("Running, Ctrl+C to quit")
    rospy.loginfo('Follow Joint Trajectory Motion Planning')
    trajectory = Trajectory()
    print("Trajectory defined")
    trajectory.__init__()
    print("Parameters defined")
    [_,pos,vel,acc,_] = trajectory.read_data()
    print("Data Reading finished")
    print("print first position", pos)
    #print("... print second data ...", pos[1])
    trajectory.add_point(pos[0], vel, acc, 1.0)
    #strajectory.add_point(pos[1], vel, acc, 10.0)
    #trajectory.add_point(pos[1], vel, acc, 10.0)
    trajectory.start()
    print("Goal sent")
    trajectory.wait(15)
    print("Follow Joint Trajectory Successfully Completed...")
    rospy.spin()