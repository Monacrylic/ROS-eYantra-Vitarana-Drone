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

    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 10  #Hertz

        #(Latitude, longitude, altitude)
        self.drone_gps_coordinates = [0.0, 0.0, 0.0]

        # Required GPS setpoint
        #(latitude, longitude, altitude)
        self.drone_gps_setpoint = [[19.0000271085, 72.0, 3], [19.0, 72.0, 3] , [19.0, 72.0, 0.0], [19.0, 72.0, 3]]
        #self.drone_gps_setpoint = [[19.0, 72.0, 3], [19.0000451704, 72.0, 3] , [19.0000451704, 72.0, 0]]
        self.setpoint_number = 0


        #max variation in any of the setpoints
        #must not be too high else the drone would topple
        # (roll, pitch , yaw)
        self.max_setpoint_variation= [5.0, 5.0, 5.0]

        # Error in position = setpoint - current location
        #(latitude, longitude, altitude)
        self.position_error= [0.0, 0.0, 0.0]

        # Error thresholds
        # (latitude, longitude, altitude)
        self.error_threshold= [0.000004517 ,0.0000047487 ,0.2]

        # Equilibrium value
        # (Roll, Pitch, Yaw, throttle)
        self.cmd_equilibrium_value= [1500, 1500, 1500, 1499]

        # (PID errors)
        # (Roll, pitch, Yaw, throttle)
        self.p_error=[0.0, 0.0, 0.0 , 0.0]
        self.i_error=[0.0, 0.0, 0.0 , 0.0]
        self.d_error=[0.0, 0.0, 0.0 , 0.0]



        #kp, ki , kd
        #Roll, pitch, yaw, throttle
        self.kp= [699, 699, 0.0, 10.52]
        self.ki= [0.0, 0.0, 0.0, 0.0]
        self.kd= [2700, 3406, 0.0, 20]


        # Prescalers
        # p-i-d
        self.throttle_prescaler= [1, 0.0, 100]
        self.roll_prescaler=[0.001, 0.0, 0.1]
        self.pitch_prescaler=[0.001, 0.0, 0.1]
        self.yaw_prescaler=[0.0, 0.0, 0.0]

        #Throttle previous error
        #(latitude, longitude, altitude)
        self.prev_error=[0.0, 0.0, 0.0]

        #total PID error
        #(roll, pitch, yaw, throttle)
        self.total_pid_error= [0.0, 0.0, 0.0, 0.0]

        #Drone Command
        self.drone_command= edrone_cmd()
        self.drone_command.rcPitch=1500.0
        self.drone_command.rcRoll=1500.0
        self.drone_command.rcYaw=1500.0
        self.drone_command.rcThrottle=1500.0

        # Drone command Min Values
        #( roll, pitch, yaw, throttle)
        self.drone_cmd_min= [1400, 1400, 0, 1000]

        # Drone command Max Values
        #( roll, pitch, yaw, throttle)
        self.drone_cmd_max= [1600, 1600, 0, 2000]

        # Subscriptions----------------------------------------------------------
        rospy.Subscriber('/edrone/gps', NavSatFix, self.drone_gps_callback)
        #rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_pid_callback)
        #------------------------------------------------------------------------

        # Publications-----------------------------------------------------------
        self.edrone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd ,queue_size=1)
        self.latitude_error= rospy.Publisher('std_msgs/latitude_error', Float32, queue_size=1)
        #------------------------------------------------------------------------

    # def roll_pid_callback(self,msg):
    #     self.kp[0]=msg.Kp * 0.001
    #     self.ki[0]=msg.Ki * 0
    #     self.kd[0]=msg.Kd * 0.1

    def drone_gps_callback(self, msg):

        self.drone_gps_coordinates[0]=msg.latitude
        self.drone_gps_coordinates[1]=msg.longitude
        self.drone_gps_coordinates[2]=msg.altitude

    def position_control(self):

        # 1. Calculate the error in setpoints
        for i in range(0,3):
            self.position_error[i]= self.drone_gps_setpoint[self.setpoint_number][i] - self.drone_gps_coordinates[i]


        #------------------------------------------------------------------------------------------------
        # Calculate Roll PID error (longitude mapping in this case)
        self.p_error[0] = self.position_error[1]
        #Integral error
        self.i_error[0] += self.position_error[1]* self.sample_time
        # Derivative error
        self.d_error[0] = (self.position_error[1]- self.prev_error[1])/ self.sample_time

        # Calculate Pitch PID error (longitude mapping in this case)
        self.p_error[1] = self.position_error[0]
        #Integral error
        self.i_error[1] += self.position_error[0]* self.sample_time
        # Derivative error
        self.d_error[1] = (self.position_error[0]- self.prev_error[0])/ self.sample_time

        # Calculate Throttle PID error
        #Proportional error
        self.p_error[3] = self.position_error[2]
        #Integral error
        self.i_error[3] += self.position_error[2] * self.sample_time
        # Derivative error
        self.d_error[3] = (self.position_error[2]- self.prev_error[2])/ self.sample_time

        #--------------------------------------------------------------------------------------------

        # Calcultae PID errors

        # Roll Pid
        self.total_pid_error[0]= (self.roll_prescaler[0]* self.p_error[0]* self.kp[0] + self.roll_prescaler[2] * self.d_error[0] * self.kd[0]) * ( 10 ** 5) #I term excluded

        # Pitch Pid
        self.total_pid_error[1]= (self.pitch_prescaler[0] * self.p_error[1]* self.kp[1] +  self.pitch_prescaler[2]* self.d_error[1] * self.kd[1]) * (10 ** 5) #I-term excluded

        # Throttle Pid
        self.total_pid_error[3]= self.throttle_prescaler[0] * self.kp[3] * self.p_error[3] + self.throttle_prescaler[1] * self.ki[3] * self.i_error[3]+ self.throttle_prescaler[2] * self.kd[3] * self.d_error[3]
        #---------------------------------------------------------------------------------------------------

        # Add errors to drone command about an equlibrium point
        self.drone_command.rcRoll= self.cmd_equilibrium_value[0] + self.total_pid_error[0]
        self.drone_command.rcPitch= self.cmd_equilibrium_value[1] + self.total_pid_error[1]
        self.drone_command.rcThrottle= self.cmd_equilibrium_value[3] + self.total_pid_error[3]

        #---------------------------CLAMP VALUES------------------------------------------------------

        def clamp(n, minn, maxn):
            if n < minn:
                return minn
            elif n > maxn:
                return maxn
            else:
                return n

        self.drone_command.rcThrottle=clamp(self.drone_command.rcThrottle, self.drone_cmd_min[3], self.drone_cmd_max[3])
        self.drone_command.rcRoll=clamp(self.drone_command.rcRoll, self.drone_cmd_min[0], self.drone_cmd_max[0])
        self.drone_command.rcPitch=clamp(self.drone_command.rcPitch, self.drone_cmd_min[1], self.drone_cmd_max[1])

        #---------------------------------------------------------------------------------------------


        # Set prev errors
        for i in range(0,3):
            self.prev_error[i]= self.position_error[i]

        self.edrone_cmd_pub.publish(self.drone_command)

        # Change the setpoint, once the target is reached
        if( (abs(self.position_error[0])< self.error_threshold[0]) and (abs(self.position_error[1])< self.error_threshold[1]) and (abs(self.position_error[2])< self.error_threshold[2]) and self.setpoint_number<=3 ):
            self.setpoint_number+=1

        # if(self.setpoint_number>=3):
        #     self.setpoint_number=3
        #print(self.setpoint_number)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_time)
    while not rospy.is_shutdown():
        e_drone.position_control()
        r.sleep()
