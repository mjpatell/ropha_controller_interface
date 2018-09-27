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


#def parse_file(self, shyam_json_1):
    # open recorded file
with open("shyam_json_1.json", 'r') as f:
    data = json.load(f)
    for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:
            # read joint names specified in file
        joint_names = item['name_']
        print (joint_names)
    print ("\n\n\n\n")

    for i in range(len(data['Template']['motions_'][0]['references_']['states_'])):
        for item in data['Template']['motions_'][0]['references_']['states_'][0]['joints_']:
            points_data = item['value_']
            print (points_data)
        print ("\n \n \n")

#def write_jointnames(self):
   #print(parse_file(self))

#def write_positions(self):
 #   print(self.points_data)


