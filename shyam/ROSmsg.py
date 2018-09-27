import roslib
import rospy
import math
import sys
import string
import actionlib
import logical
import trajectory_msgs.msg

from sensor_msgs.msg import JointState
from actionlib import simple_action_client
from pr2_controllers_msgs.msg import JointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64
from logical import Cluster
from joy import * 