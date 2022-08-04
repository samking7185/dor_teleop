# dor_teleop

Repository for teleop control of the door opening robot

The robot has a total of 5 motors.

The 3 motors on the door opening device are subscribed to topics /robo1_cmd, /robo2_cmd, /robo3_cmd These motors receive messages of type, std_msgs/Int32.

The 2 motors on the mobile base are controlled by a Pololu motor driver and only require 1 topic, /cmd_vel. These motors receive messages of type, geometry_msgs/Twist.
