#!/usr/bin/env python

'''
This is a demo script for activating the gripper whenever a box is detected
'''

from std_msgs.msg import String
import rospy
from vitarana_drone.srv import Gripper

class activate_gripper():

	# initializing everything
	def __init__(self):
		rospy.init_node("gripper_activator")		
		self.box_present = rospy.Subscriber("/edrone/gripper_check", String,self.box_check_callback)
		self.gripper = rospy.Subscriber("Gripper",Gripper,self.gripper_callback)
		self.box_present = False


	def box_check_callback(self,value):		
		if value.data=="True":
			self.box_present = True
		
	def gripper_callback(self,req):
		if(self.box_present==True):
			pass



if __name__ == '__main__':
	activate_gripper = activate_gripper()
	rospy.spin()
	
