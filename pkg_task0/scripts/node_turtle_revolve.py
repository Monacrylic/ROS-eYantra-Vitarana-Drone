#!/usr/bin/env python

##Task 0
#To know when its completed a revolution it checks the reading of theta, which if changes from negative to po


import rospy

#Check the ROS website to know from where to import which package
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import sys


stop_sim= False;
first_reading= True;
prev_angle=0


#callback for the pose subscriber
def pose_data(pose):
	global stop_sim, first_reading, start_angle,start_x, start_y, prev_angle

	#-----------------------------Take initial readings-----------------------
	if(first_reading):
		start_angle=pose.theta
		start_x=pose.x
		start_y=pose.y
		rospy.loginfo("PositionX = %f: PositionY = %f: angle: %f", start_x, start_y, pose.theta)
		first_reading=False
	#------------------------------------------------------------------------

	rospy.loginfo("Angle travelled: %f", pose.theta)
	if(prev_angle<0 and pose.theta>0):
		stop_sim=True
	prev_angle=pose.theta



#main function
def node_turtle_revolve(lin_vel= 3.0,ang_vel=1.5):

	rospy.init_node('move_turtle', anonymous=True)
	rospy.Subscriber('/turtle1/pose',Pose, pose_data)

	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(10) # refresh screen every 1/10s

	vel = Twist()
	radius= round(lin_vel/ang_vel)
	rospy.loginfo("Radius of revolution: %f", radius)


	while not rospy.is_shutdown():


		vel.linear.y = 0
		vel.linear.z = 0
		vel.linear.x = lin_vel

		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = ang_vel


		pub.publish(vel)

		if(stop_sim):
				rospy.loginfo("Completed Revolution")
				break

		rate.sleep()

if __name__ == '__main__':
	try:
		#node_turtle_revolve(float(sys.argv[1]),float(sys.argv[2]))
		node_turtle_revolve()
	except rospy.ROSInterruptException:
		pass
