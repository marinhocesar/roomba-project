#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from gpiozero import CamJamKitRobot, Servo
from time import sleep

class cam:
    def __init__(self):
        self.robot = CamJamKitRobot()
        self.sub1 = rospy.Subscriber('/controller/cmd_head_hor', Float32, self.Horizontal)
        self.sub2 = rospy.Subscriber('/controller/cmd_head_ver', Float32, self.Vertical)

    def Horizontal(self,msg):
        servo = Servo(24)
        while True:
            servo.min()
            sleep(1)
            servo.mid()
            sleep(1)
            servo.max()
            sleep(1)

    def Vertical(self,msg):
        servo = Servo(25)
        while True:
            servo.min()
            sleep(1)
            servo.mid()
            sleep(1)
            servo.max()
            sleep(1)


if __name__ == "__main__":

    rospy.init_node("head_controller")
    c = cam()
    rospy.spin()
