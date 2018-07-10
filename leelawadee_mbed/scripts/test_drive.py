#!/usr/bin/env python
import rospy
import serial
from geometry_msgs.msg import Twist

# ser = serial.Serial('/dev/ttyUSB0',baudrate=115200)

goal_linear_velocity = 0.0
goal_angular_velocity = 0.0

def setSpeedRaw(RawSpeedL,RawSpeedR):
    command = "#1P%s#2P%sT500" % (str(RawSpeedL), str(RawSpeedR))
    rospy.loginfo(command)
    # print(command)

def cmdvelCB(msg):
    global goal_linear_velocity,goal_angular_velocity
    goal_linear_velocity = msg.linear.x
    goal_angular_velocity = msg.angular.z

def timerCB(event):
    global goal_linear_velocity,goal_linear_velocity
    speedL = goal_linear_velocity - (goal_angular_velocity * 0.2)
    speedR = goal_linear_velocity + (goal_angular_velocity * 0.2)
    speedL = 1500 + speedL * 1000/(2.19*0.15)
    speedR = 1500 + speedR * 1000/(2.19*0.15)
    setSpeedRaw(int(speedL) ,int(speedR))

def main():
    rospy.init_node('Test_Drive')
    rospy.Subscriber('/cmd_vel',Twist,cmdvelCB)
    timer = rospy.Timer(rospy.Duration(0.5),timerCB)
    rospy.spin()
    timer.shutdown()

    # setSpeedRaw(100,500)
    # setSpeedRaw(900,2500)
    # pass

if __name__ == '__main__':
    main()