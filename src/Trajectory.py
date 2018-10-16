#!/usr/bin/python

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal, FollowJointTrajectoryActionGoal
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
    def __init__(self):
        self.object1 = FollowJointTrajectoryGoal()
        self.goal = FollowJointTrajectoryActionGoal()
        self.position = PoseStamped()
        self.point = JointTrajectoryPoint()
        self.goal_time_tolerance = rospy.Time(1)
        self.object1.goal_time_tolerance = self.goal_time_tolerance
        self.client = actionlib.SimpleActionClient('/arm_left/joint_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)


    def create_frame(self, sequence = '', stamp_secs = 0, stamp_nsecs = 0, frame_id = ''):
        self.object1.trajectory.header.seq = sequence
        self.object1.trajectory.header.stamp.secs = stamp_secs
        self.object1.trajectory.header.stamp.nsecs = stamp_nsecs
        self.object1.trajectory.header.frame_id = frame_id

        #return sequence, stamp_secs, stamp_nsecs, frame_id
        return self.object1

    """
    def fill_trajectory(self):

        [jointnames, pointposition, pointvelocity, pointacceleration, duration] = self.read_data()
        self.object1.trajectory.joint_names = jointnames

        for pose, vel, acc in [pointposition, pointvelocity, pointacceleration]:
            pose =


        self.object1.trajectory.points.positions = pointposition
        self.object1.trajectory.points.velocities = pointvelocity
        self.object1.trajectory.points.acceleration = pointacceleration
        self.object1.trajectory.points.time_from_start = duration

        #return jointnames, pointposition, pointvelocity, pointacceleration, duration
        return self.object1

    """
    """
    def trajectory_points(self, position, velocity, acceleration, time_from_start):

        self.position.pose.position.x = float(position[0])
        self.position.pose.position.y = float(position[1])
        self.position.pose.position.z = float(position[2])
        self.position.pose.orientation.x = float(position[3])
        self.position.pose.orientation.y = float(position[4])
        self.position.pose.orientation.z = float(position[5])
        self.position.pose.orientation.w = float(position[6])


        self.fill_trajectory()
        return self.position
    """

    def add_point(self, positions = '', time=0.0):
        self.point.positions = positions
        self.point.time_from_start = rospy.Duration(time)
        self.object1.trajectory.points.append(self.point)
        return self.point, self.object1, positions, time

    def start(self):
        self.object1.trajectory.header.stamp = rospy.Time.now()
        self.client.send_goal(self.object1)

    def stop(self):
        self.client.cancel_goal()

    def goal_id(self, goal_id='', goal_stamp_secs=0.0, goal_stamp_nsecs=0.0):
        self.goal.goal_id.id = goal_id
        self.goal.goal_id.stamp.secs = goal_stamp_secs
        self.goal.goal_id.stamp.nsecs = goal_stamp_nsecs
        return self.goal

    def path_tolerance(self, path_tol):
        self.object1.path_tolerance = path_tol
        print(path_tol)

        return self.object1

    def goal_tolerance(self, goal_tol):
        self.object1.goal_tolerance = goal_tol
        print(goal_tol)

        return self.object1

    def wait(self, timeout):
        self.client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self.client.get_result()

    def write_jointnames(self, joint_name, point_positions):
        with open('shyam.txt', 'w') as f:
            json.dump("Joint_Names:", f)
            json.dump(joint_name, f)
            f.write('\n \n')
            f.write("Position Data for the Points:")
            f.write('\n')
            json.dump(point_positions, f)
            f.write('\n\n')

    def write_velocity(self, velocity, acceleration):
        with open('shyam.txt', 'a') as f:
            json.dump("Velocity:", f)
            json.dump(velocity, f)
            f.write('\n\n')
            json.dump("Acceleration:", f)
            json.dump(acceleration, f)
            f.write('\n\n')

    def read_data(self):
        with open('shyam_json_1.json') as f:
            data = json.load(f)
            joint_name = []; position_values = []
            for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:        #read joint names specified in file
                self.object1.trajectory.joint_names.append(item['name_'])

            for i in range(len(data['Template']['motions_'][0]['references_']['states_'])):
                c = []
                for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:
                    c.append(item['value_'])
                position_values.append(c)
                #self.point.positions.append(position_values)

            self.write_jointnames(joint_name, position_values)
            #self.trajectory_points(c)
            self.add_point(position_values, 0.0)
        #write_positions(b)
        #for item in range(len(data['Template']['motions_'][0]['properties_']['dynamics_'])):

            d = []; e = []
            vel = data['Template']['motions_'][0]['properties_']['dynamics_']['velocities_']
            acc = data['Template']['motions_'][0]['properties_']['dynamics_']['accelerations_']
            d.append(vel)
            e.append(acc)
            self.point.velocities.append(vel)
            self.point.accelerations.append(acc)
            #self.fill_trajectory(joint_name, position_values, vel, acc, None)
        return [joint_name, position_values, vel, acc, 0.0]
	

if __name__ == '__main__':
    #parser = argparse.ArgumentParser()
    #parser.add_argument('--base_link', required=True, default='base_link', help='base link of the kinematic chain')

    #args, unknown = parser.parse_known_args()

    print("Initializing the Node...")
    rospy.init_node("follow_joint_trajectory_goal")
    print("Running, Ctrl+C to quit...")

    rospy.loginfo('Follow Joint Trajectory Motion Planning')

    trajectory = Trajectory()
    trajectory.__init__()
    trajectory.add_point()
    trajectory.read_data()
    trajectory.create_frame()
    trajectory.goal_id()
    trajectory.start()
    trajectory.wait(2)
    print("Follow Joint Trajectory Successfully Completed...")
