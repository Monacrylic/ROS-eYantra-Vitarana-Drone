#!/usr/bin/env python
'''
This the Position contoller for task1
TODO: Tune PID!
'''
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Float64
import rospy
import time
import math
import tf

class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.50  # in seconds

        #(Latitude, longitude, altitude)
        self.drone_gps_coordinates = [0.0, 0.0, 0.0]

        # Required GPS setpoint
        #(latitude, longitude, altitude)
        self.drone_gps_setpoint = [0.0, 0.0, 1]

        # Error in position = setpoint - current location
        #(latitude, longitude, altitude)
        self.position_error= [0.0, 0.0, 0.0]

        #Drone Command

        self.drone_command= edrone_cmd()
        self.drone_command.rcPitch=0.0
        self.drone_command.rcRoll=0.0
        self.drone_command.rcYaw=0.0
        self.drone_command.rcThrottle=0.0

        # Subscriptions----------------------------------------------------------
        rospy.Subscriber('/edrone/gps', NavSatFix, self.drone_gps_callback)
        #------------------------------------------------------------------------

        # Publications-----------------------------------------------------------
        self.edrone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd ,queue_size=1)
        #------------------------------------------------------------------------

    def drone_gps_callback(self, msg):

        self.drone_gps_coordinates[0]=msg.latitude
        self.drone_gps_coordinates[1]=msg.longitude
        self.drone_gps_coordinates[2]=msg.altitude

    def position_control(self):

        # 1. Calculate the error in setpoints
        for i in range(0,3):
            self.position_error[i]= self.drone_gps_setpoint[i] - self.drone_gps_coordinates[i]

        # 2. Move the drone to neutralize the error

        self.drone_command.rcRoll= 1500;
        self.drone_command.rcPitch= 1500;
        self.drone_command.rcYaw= 1500;

        if(self.position_error[2]< 0):
            self.drone_command.rcThrottle=1495
        else:
            self.drone_command.rcThrottle= 1505


        self.edrone_cmd_pub.publish(self.drone_command)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_time)
    while not rospy.is_shutdown():
        e_drone.position_control()
        r.sleep()
