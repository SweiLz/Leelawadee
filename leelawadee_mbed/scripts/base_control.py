#!/usr/bin/env python

import rospy
import tf
import sys
import serial
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class BaseControl(object):
    def __init__(self):
        self.baseId = rospy.get_param("~base_id", "base_footprint")
        self.odomId = rospy.get_param("~odom_id", "odom")
        self.port = rospy.get_param("~port", "/dev/ttySTM32")
        self.baudrate = long(rospy.get_param("~baudrate", "115200"))
        self.wheelSep = float(rospy.get_param("~wheel_separation", "0.5"))
        self.wheelRad = float(rospy.get_param("~wheel_radius", "0.102"))
        self.MAX_W = float(rospy.get_param("~wheel_speed", "2.136283002"))

        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.odom_freq = float(rospy.get_param("~odom_freq", "50"))
        self.cmd_freq = float(rospy.get_param("~cmd_freq", "10"))

        try:
            self.serial = serial.Serial(self.port,self.baudrate,timeout=10)
        except serial.serialutil.SerialException:
            rospy.logerr("Cannot connect to port: " + self.port + ".")
            sys.exit(0)
        rospy.loginfo("Communication success!")

        self.sub = rospy.Subscriber(
            "cmd_vel", Twist, self.cmdCB, queue_size=10)
        self.timer_cmd = rospy.Timer(rospy.Duration(
            1.0/self.cmd_freq), self.timerCmdCB)

        self.trans_x = 0.0
        self.rotat_z = 0.0

    def cmdCB(self, msg):
        self.trans_x = msg.linear.x
        self.rotat_z = msg.angular.z

    def constrain(self, value, value_min, value_max):
        return max(min(value_max, value), value_min)

    def timerCmdCB(self, event):
        self.sendWL = self.constrain(
            (self.trans_x - self.wheelSep/2.0*self.rotat_z)/self.wheelRad, -self.MAX_W, self.MAX_W)
        self.sendWR = self.constrain(
            (self.trans_x + self.wheelSep/2.0*self.rotat_z)/self.wheelRad, -self.MAX_W, self.MAX_W)
        speedL = self.constrain(1500 + self.sendWL *
                                1000.0/self.MAX_W, 500, 2500)
        speedR = self.constrain(1500 - self.sendWR *
                                1000.0/self.MAX_W, 500, 2500)
        command = "#1P{}#2P{}T1\r\n".format(int(speedL), int(speedR))
        # rospy.logwarn(command)
        self.serial.write(command)


if __name__ == '__main__':
    try:
        rospy.init_node("base_control")
        rospy.loginfo("Leelawadee Base Control ...")
        bc = BaseControl()
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
        print("Shutting down")
