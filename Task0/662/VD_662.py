#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

theta = 0.0


#funtion to get the updated theta values
def update_pose(data):
	global theta
	pose = data
	#rospy.loginfo("from update pose {}".format(data.theta))
	theta = round(pose.theta,4)
	
#function to make the turtle revolve
def rotate():
	rospy.init_node("move_turtle", anonymous=True)
	velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) #pubblishing the velocity to the turtlesim simulator
	pose_subsriber = rospy.Subscriber('/turtle1/pose',Pose,update_pose) 
	vel_msg = Twist()
	pose = Pose()	
	
	#setting the linear and angular speed
	vel_msg.linear.x = 0.7
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0.0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0.5
	
	
	#breaking the loop when the turtle is at an angle of greater than -0.4(almost 360 deg) and less than 0.0 (0 degrees)
	while(True):
		print("The angle covered is {}".format(theta))				
		velocity_publisher.publish(vel_msg)
		if(theta>-0.4 and theta<0.0):			
			break
		

	print("Goal reached!!")
		
	rospy.spin()
		

#main function for accessing the functions in the file.
if __name__ == "__main__":
	try:
		rotate()
	except rospy.ROSInterruptException:
		 pass
