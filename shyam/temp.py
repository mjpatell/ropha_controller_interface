import unittest
import json
#import rospy
#import rostest
#from pprint import pprint
#from rospy_message_converter import message_converter

with open('shyam.json') as f:
    data = json.load(f)

class TestMessageConverter(unittest.TestCase):
    #def define_data(self):
        #with open('shyam.json') as f:
           # data = json.load(f)

    def test_ros_message_with_header(self):
        from std_msgs.msg import Header
        rospy.init_node('time_node')
        now_time = rospy.Time.now()
        expected_dictionary = {
            'stamp': { 'secs': now_time.secs, 'nsecs': now_time.nsecs },
            'frame_id' : 'my_frame',
            'seq': 3
        }
        message = Header(
            stamp = now_time,
            frame_id = expected_dictionary['frame_id'],
            seq = expected_dictionary['seq']
        )
        dictionary = message_converter.convert_ros_message_to_dictionary(message)
        self.assertEqual(dictionary, expected_dictionary)
        
    def test_ros_message_with_child_message(self):
        from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
        
        joint1_position = data['Template']['motions_'][0]['references_']['states_'][0]['joints_'],
        
        joint2_position = data['Template']['motions_'][0]['references_']['states_'][1]['joints_'],
        print (joint1_position['value_'], joint2_position['value_'])
        #layout = MultiArrayLayout(
         #   dim = [dimension1, dimension2],
          #  data_offset = expected_dictionary['layout']['data_offset']
        #)
        #multiArray = Float64MultiArray(
         #   layout = layout,
          #  data   = expected_dictionary['data']
        #)
        dictionary = message_converter.convert_ros_message_to_dictionary(multiArray)
        self.assertEqual(dictionary, expected_dictionary)