#!/usr/bin/env python
'''
This the Position contoller for task1
Ranges-
Latitude -> -90 to 90
Longitude -> -180 t0 180
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
        self.sample_time = 20  #Hertz

        #(Latitude, longitude, altitude)
        self.drone_gps_coordinates = [0.0, 0.0, 0.0]

        # Required GPS setpoint
        #(latitude, longitude, altitude)
        self.drone_gps_setpoint = [0.0, 0.0, 3]

        # RPY setpoint correction term
        #This term is aded to the setpoint for RPY and throttle
        #(roll, pitch, yaw, throttle)
        self.drone_cmd_correction_term=[0.0, 0.0 , 0.0, 0.0]

        #max variation in any of the setpoints
        #must not be too high else the drone would topple
        # (roll, pitch , yaw, throttle)
        self.max_setpoint_variation= [10.0, 10.0, 10.0, 50]

        # Error in position = setpoint - current location
        #(latitude, longitude, altitude)
        self.position_error= [0.0, 0.0, 0.0]

        # Equilibrium value

        self.cmd_equilibrium_value= 1499

        #
        self.throttle_pid_error= [0.0, 0.0, 0.0]

        #kp, ki , kd
        self.throttle_pid= [10.92 , 0.0, 100]

        # Prescalers
        self.prescaler= [1, 0.0, 100]

        #Throttle previous error
        self.throttle_prev_error= 0.0

        #total error
        self.total_pid_error= 0.0
        #Drone Command

        self.drone_command= edrone_cmd()
        self.drone_command.rcPitch=1500.0
        self.drone_command.rcRoll=1500.0
        self.drone_command.rcYaw=1500.0
        self.drone_command.rcThrottle=1500.0

        # Subscriptions----------------------------------------------------------
        rospy.Subscriber('/edrone/gps', NavSatFix, self.drone_gps_callback)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_pid_callback)
        #------------------------------------------------------------------------

        # Publications-----------------------------------------------------------
        self.edrone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd ,queue_size=1)
        self.altitude_error= rospy.Publisher('std_msgs/altitude_error', Float32, queue_size=1)
        #------------------------------------------------------------------------

    def altitude_pid_callback(self,msg):
        self.throttle_pid[0]=msg.Kp * 0.01
        self.throttle_pid[1]=msg.Ki * 0.001
        self.throttle_pid[2]=msg.Kd * 2

    def drone_gps_callback(self, msg):

        self.drone_gps_coordinates[0]=msg.latitude
        self.drone_gps_coordinates[1]=msg.longitude
        self.drone_gps_coordinates[2]=msg.altitude

    def position_control(self):

        # 1. Calculate the error in setpoints
        for i in range(0,3):
            self.position_error[i]= self.drone_gps_setpoint[i] - self.drone_gps_coordinates[i]


        # 2. Calculate Throttle PID error

        #Proportional error
        self.throttle_pid_error[0]= self.position_error[2]

        #Integral error
        self.throttle_pid_error[1] += self.position_error[1]* self.sample_time

        # Derivative error
        self.throttle_pid_error[2]= (self.position_error[2]- self.throttle_prev_error)/ self.sample_time




        # 3. Calculate correction terms

        # Deal with throttle first
        self.total_pid_error= self.prescaler[0] * self.throttle_pid[0] * self.throttle_pid_error[0] + self.prescaler[1] * self.throttle_pid[1] * self.throttle_pid_error[1]+ self.prescaler[2] * self.throttle_pid[2] * self.throttle_pid_error[2]
        self.drone_command.rcThrottle= self.cmd_equilibrium_value + self.total_pid_error

        #print(self.total_pid_error);


        def clamp(n, minn, maxn):
            if n < minn:
                return minn
            elif n > maxn:
                return maxn
            else:
                return n


        self.drone_command.rcThrottle=clamp(self.drone_command.rcThrottle, 1000, 2000)
        #Roll
        #self.drone_cmd_correction_term[0]= clamp(self.position_error[0]*10 ,-1 * self.max_setpoint_variation[0], self.max_setpoint_variation[0])
        #Pitch
        #self.drone_cmd_correction_term[1]= clamp(self.position_error[1]*10 ,-1 * self.max_setpoint_variation[1], self.max_setpoint_variation[1])
        #throttle
        #self.drone_cmd_correction_term[3]= clamp(self.position_error[2]*100 ,-30, 20)
        # 3. Add the correction to the drone_command

        #self.drone_command.rcRoll= self.cmd_equilibrium_value + self.drone_cmd_correction_term[0]
        #self.drone_command.rcPitch= self.cmd_equilibrium_value + self.drone_cmd_correction_term[1]





        self.throttle_prev_error= self.throttle_pid_error[0]

        self.altitude_error.publish(self.total_pid_error)
        self.edrone_cmd_pub.publish(self.drone_command)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_time)
    while not rospy.is_shutdown():
        e_drone.position_control()
        r.sleep()
