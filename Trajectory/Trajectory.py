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
import time
#header = roslib.msg.Header()
#frame = header.frame_id
#header.stamp = rospy.get_rostime()


#def joint_names(joint_name):
    #print(joint_name)

#def point_data(point_positions):
    #print(point_positions)


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
