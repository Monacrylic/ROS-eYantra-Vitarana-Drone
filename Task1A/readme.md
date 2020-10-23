Terminal commands in order-

1) roslaunch vitarana_drone drone.launch

2) rosrun vitarana_drone attitude_controller.py

3) rosrun pid_tune pid_tune_drone.py

4) rostopic pub /drone_command vitarana_drone/edrone_cmd "{rcRoll: 1500.0, rcPitch: 1500.0, rcYaw: 1500.0, rcThrottle: 1505.0, aux1: 0.0, aux2: 0.0, aux3: 0.0,
  aux4: 0.0}"
