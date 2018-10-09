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
from geometry_msgs.msg import *
#import control_msgs
import roslib
import json
import argparse


class Trajectory:
    def start(self):
        self.goal.trajectory.header.stamp = rospy.Time.now()
        self.client.send_goal(self.goal)


#def joint_names(joint_name):
    #print(joint_name)

#def point_data(point_positions):
    #print(point_positions)

    def create_frame(self, sequence, stamp_secs, stamp_nsecs, frame_id):
        pos = FollowJointTrajectoryGoal()
        pos.trajectory.header.seq = self.sequence
        pos.trajectory.header.stamp.secs = self.stamp_secs = rospy.Time.secs
        pos.trajectory.header.stamp.nsecs = self.stamp_nsecs = rospy.Time.nsecs
        pos.trajectory.header.frame_id = self.frame_id

    def positions(self, jointnames, pointposition, pointvelocity, pointacceleration, duration):
        pos = FollowJointTrajectoryGoal()
        pos.trajectory.joint_names = self.jointnames
        pos.trajectory.points.positions = self.pointposition
        pos.trajectory.points.velocities = self.pointvelocity
        pos.trajectory.points.acceleration = self.pointacceleration
        pos.trajectory.points.time_from_start = self.duration = rospy.Duration

        return pos

    def position_vector(self, p = []):
        self.p = self.write_jointnames(point_positions=[])
        position = PoseStamped()
        position.pose.position.x = float(p[0])
        position.pose.position.y = float(p[1])
        position.pose.position.z = float(p[2])
        position.pose.orientation.x = float(p[3])
        position.pose.orientation.y = float(p[4])
        position.pose.orientation.z = float(p[5])
        position.pose.orientation.w = float(p[6])

        return position


    def follow_traj_client(pos):
        client = actionlib.ActionClient
        client.wait_for_server(timeout=2.0)
        client.send_goal(pos)
        client.wait_for_server(timeout=5.0)
        return client.get_result()

    def goal_ID(self, goal_id, goal_stamp):
        goal = FollowJointTrajectoryActionGoal()
        goal.goal_id.id = self.goal_id
        goal.goal_id.stamp = self.goal_stamp

    def path_tolerence(self):
        path_tol = FollowJointTrajectoryGoal.goal_tolerance
        print(path_tol)

    def goal_tolerence(self):
        goal_tol = FollowJointTrajectoryGoal.goal_tolerance
        print(goal_tol)

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
            a = []; b = []
            for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:        #read joint names specified in file
                a.append(item['name_'])
            #joint_names(a)

            for i in range(len(data['Template']['motions_'][0]['references_']['states_'])):
                c = []
                for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:
                    c.append(item['value_'])
                b.append(c)
        #point_data(b)
            self.write_jointnames(self, a, b)
        #write_positions(b)
        #for item in range(len(data['Template']['motions_'][0]['properties_']['dynamics_'])):
            d = []; e = []
            vel = data['Template']['motions_'][0]['properties_']['dynamics_']['velocities_']
            acc = data['Template']['motions_'][0]['properties_']['dynamics_']['accelerations_']
            d.append(vel)
            e.append(acc)
            self.write_velocity(self, vel, acc)


#def write_jointnames(self):
        #print(ReadData.parse_file())

#def write_positions(self):
    #print(parse_file(self.points_data))

    if __name__ == '__main__':
        rospy.init_node("follow_joint_trajectory_goal")

        parser = argparse.ArgumentParser()
        parser.add_argument('--base_link', required=True, default='base_link', help='base link of the kinematic chain')

        args, unknown = parser.parse_known_args()

