#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

theta = 0.0

def update_pose(data):
	global theta
	pose = data
	#rospy.loginfo("from update pose {}".format(data.theta))
	theta = round(pose.theta,4)
	

def rotate():
	rospy.init_node("move_turtle", anonymous=True)
	velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	pose_subsriber = rospy.Subscriber('/turtle1/pose',Pose,update_pose)
	vel_msg = Twist()
	pose = Pose()	
	
	vel_msg.linear.x = 0.7
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0.0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0.5
	
	count = 0
	while(True):
		print("The angle covered is {}".format(theta))				
		velocity_publisher.publish(vel_msg)
		if(theta>-0.4 and theta<0.0):			
			break
		

	print("Goal reached!!")
		
	rospy.spin()
		


if __name__ == "__main__":
	try:
		rotate()
	except rospy.ROSInterruptException:
		 pass
