# -*- coding: utf-8 -*-
"""
Created on Wed Sep 26 12:52:44 2018

@author: st154
"""
import unittest
import json

with open('shyam_json_1.json') as f:
    data = json.load(f)
    
class TestMessageConverter(unittest.TestCase):
    def test_ros_message_with_child_message(self):
        #from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
        
        joint1_position = data['Template']['motions_'][0]['references_']['states_'][0]['joints_'],
        
        joint2_position = data['Template']['motions_'][0]['references_']['states_'][1]['joints_'],
        print (joint1_position['value_'], joint2_position['value_'])
        
test_ros_message_with_child_message(self)