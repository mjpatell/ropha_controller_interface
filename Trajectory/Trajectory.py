#!/usr/bin/python

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
import geometry_msgs
#import control_msgs
import roslib
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import json
import time
import sys

#header = roslib.msg.Header()
#frame = header.frame_id
#header.stamp = rospy.get_rostime()

def __init__(self, robot):
    ns = 'robot/limb/' + robot + '/'
    self._client = actionlib.SimpleActionClient(
        ns + "follow_joint_trajectory",
        FollowJointTrajectoryAction,
    )
    self._goal = FollowJointTrajectoryGoal()
    self._goal_time_tolerance = rospy.Time(0.1)
    self._goal.goal_time_tolerance = self._goal_time_tolerance
    server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
    if not server_up:
        rospy.logerr("Timed out waiting for Joint Trajectory"
                     " Action Server to connect. Start the action server"
                     " before running example.")
        rospy.signal_shutdown("Timed out waiting for Action Server")
        sys.exit(1)

    self.clear(robot)

def start(self):
    self.goal.trajectory.header.stamp = rospy.Time.now()
    self.client.send_goal(self.goal)





#def joint_names(joint_name):
    #print(joint_name)

#def point_data(point_positions):
    #print(point_positions)

def create_frame(sequence, stamp, frame_id):
    pos = FollowJointTrajectoryGoal()
    pos.trajectory.header.seq = sequence
    pos.trajectory.header.stamp = stamp = rospy.Time.secs, rospy.Time.nsecs
    pos.trajectory.header.frame_id = frame_id

def positions(pointposition, pointvelocity, pointacceleration, duration):
    pos = FollowJointTrajectoryGoal()
    pos.trajectory.points.positions = pointposition
    pos.trajectory.points.velocities = pointvelocity
    pos.trajectory.points.acceleration = pointacceleration
    pos.trajectory.points.time_from_start = duration = rospy.Duration

    return pos




def write_jointnames(joint_name, point_positions):
    with open('shyam.txt', 'w') as f:
        json.dump("Joint_Names:", f)
        json.dump(joint_name, f)
        f.write('\n \n')
        f.write("Position Data for the Points:")
        f.write('\n')
        json.dump(point_positions, f)
        f.write('\n\n')

def write_velocity(velocity, acceleration):
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
        for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:
            # read joint names specified in file
         a.append(item['name_'])
        #joint_names(a)

        for i in range(len(data['Template']['motions_'][0]['references_']['states_'])):
            c = []
            for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:
                c.append(item['value_'])
            b.append(c)
        #point_data(b)
        write_jointnames(a, b)
        #write_positions(b)

        #for item in range(len(data['Template']['motions_'][0]['properties_']['dynamics_'])):
        d = []; e = []
        vel = data['Template']['motions_'][0]['properties_']['dynamics_']['velocities_']
        acc = data['Template']['motions_'][0]['properties_']['dynamics_']['accelerations_']
        d.append(vel)
        e.append(acc)
        write_velocity(vel, acc)


#def write_jointnames(self):
        #print(ReadData.parse_file())

#def write_positions(self):
    #print(parse_file(self.points_data))
