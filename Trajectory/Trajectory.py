#!/usr/bin/python

#import rospy

#import actionlib
#from control_msgs.msg import (
#    FollowJointTrajectoryAction,
#    FollowJointTrajectoryGoal,
#)
#from trajectory_msgs.msg import (
#    JointTrajectoryPoint,
#)
#import geometry_msgs
#import control_msgs
#import roslib
#from geometry_msgs.msg import Pose
import json

#header = roslib.msg.Header()
#frame = header.frame_id
#header.stamp = rospy.get_rostime()


def joint_names(joint_name):
    print(joint_name)

def point_data(point_positions):
    print(point_positions)

def write_jointnames(joint_name, point_positions):
    with open("shyam.txt", "w"):
        print(joint_name)
    with open("shyam.txt", "a"):
        print(point_positions)

#def write_positions(point_positions):

with open('shyam_json_1.json') as f:
    data = json.load(f)
    a = []; b = []; c = []
    for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:
        # read joint names specified in file
        a.append(item['name_'])
    joint_names(a)

    for i in range(len(data['Template']['motions_'][0]['references_']['states_'])):
        for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:
            c.append(item['value_'])
        b.append(c)
    point_data(b)
    write_jointnames(a, b)



#def write_jointnames(self):
        #print(ReadData.parse_file())

#def write_positions(self):
    #print(parse_file(self.points_data))
