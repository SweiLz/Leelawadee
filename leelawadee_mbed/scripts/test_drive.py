#!/usr/bin/env python
import rospy
import serial
from geometry_msgs.msg import Twist

ser = serial.Serial('/dev/ttyUSB0',baudrate=115200)

WHEEL_SAPARATE = 0.5
WHEEL_RADIUS = 0.102

VELOCITY_CONSTANT = 2.136283002 # 20.4 rpm = 2.178170904 rad/s * 0.102 m

goal_linear_velocity = 0.0
goal_angular_velocity = 0.0

def constrain(value,value_min,value_max):
    return max(min(value_max, value), value_min)

def setSpeedRaw(RawSpeedL,RawSpeedR):
    command = "#1P{}#2P{}T1\r\n".format(RawSpeedL, RawSpeedR)
    rospy.loginfo(command)
    ser.write(command)

def cmdvelCB(msg):
    global goal_linear_velocity,goal_angular_velocity
    goal_linear_velocity = msg.linear.x
    goal_angular_velocity = msg.angular.z

def timerCB(event):
    global goal_linear_velocity,goal_linear_velocity
    speedL = goal_linear_velocity - (goal_angular_velocity * WHEEL_SAPARATE/2)
    speedR = goal_linear_velocity + (goal_angular_velocity * WHEEL_SAPARATE/2)
    rospy.loginfo("{}, {}".format(speedL,speedR))
    speedL = 1500 + speedL * 1000/(VELOCITY_CONSTANT*WHEEL_RADIUS)
    speedR = 1500 - speedR * 1000/(VELOCITY_CONSTANT*WHEEL_RADIUS)
    speedL = int(constrain(speedL,500,2500))
    speedR = int(constrain(speedR,500,2500))
    setSpeedRaw(speedL ,speedR)

def main():
    rospy.init_node('Test_Drive')
    
    rospy.Subscriber('/cmd_vel',Twist,cmdvelCB)
    # ser.write("#255PCLE\r\n")
    # rospy.sleep(1)
    # ser.write("#255PMOD7\r\n")
    # rospy.sleep(1)
    timer = rospy.Timer(rospy.Duration(0.1),timerCB)
    rospy.spin()
    timer.shutdown()

if __name__ == '__main__':
    main()
