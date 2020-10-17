#!/usr/bin/env python

##Task 0
#To do stop the simulation after one cycle


import rospy

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import sys 


stop_sim= False;
first_reading= True;


def pose_data(pose):
	global stop_sim, first_reading, start_angle
	rospy.loginfo("Angle travelled: %f", pose.theta)
	if(first_reading):
		start_angle=pose.theta
	


def node_turtle_revolve(lin_vel,ang_vel):

	rospy.init_node('move_turtle', anonymous=True)
	rospy.Subscriber('/turtle1/pose',Pose, pose_data)

	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(10) # refresh screen every 1/10s

	vel = Twist()
	
	
	while not rospy.is_shutdown():

	
		vel.linear.y = 0
		vel.linear.z = 0
		vel.linear.x = lin_vel

		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = ang_vel



		#rospy.loginfo("PositionX = %f: PositionY = %f:",posX_var, posY_var)

		pub.publish(vel)

		
		


		rate.sleep()

if __name__ == '__main__':
	try:
		node_turtle_revolve(float(sys.argv[1]),float(sys.argv[2]))
	except rospy.ROSInterruptException:
		pass
