#!/usr/bin/env python

'''
This is a demo script for activating the gripper whenever a box is detected
'''

from std_msgs.msg import String
import rospy
from vitarana_drone.srv import Gripper
import time

class Edrone():

	# initializing everything
	def __init__(self):
		rospy.init_node("gripper_activator")		
		self.box_present = rospy.Subscriber("/edrone/gripper_check", String,self.box_check_callback)
		self.box_present = False		
		self.gripper = rospy.ServiceProxy('edrone/activate_gripper',Gripper)
		self.sample_time = 10		

	def box_check_callback(self,value):		
		if value.data=="True":
			self.box_present = True
		


	def main(self):
		if(self.box_present==True):			
			self.gripper(True)

		
		
		




if __name__ == '__main__':
	e_drone = Edrone()
	r = rospy.Rate(e_drone.sample_time)
	while not rospy.is_shutdown():
		e_drone.main()
		r.sleep()