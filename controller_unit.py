#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from time import sleep

class uSound:
    def __init__(self):
        self.sub = rospy.Subscriber('/ultrasound_reader/distance', LaserScan, self.update)
        self.ranges = [0,0,0]

    def update(self,msg):
        self.ranges = msg.ranges

def check(us, pub, ang):
    pub.publish(ang)
    sleep(1)
    if us.ranges[1]>80:
        return True
    return False

def goForward(us, pub, cmd):
    if cmd:
        cmd = 0.003*us.ranges[1]
    pub.publish(cmd)

if __name__ == "__main__":

    rospy.init_node("controller_unit")
    pub1 = rospy.Publisher("/controller/cmd_dis", Float32, queue_size=1) #Distância
    pub2 = rospy.Publisher("/controller/cmd_ang", Float32, queue_size=1) #Angulo
    pub3 = rospy.Publisher("/controller/cmd_head_hor", Float32, queue_size=1) #Camera (Horizontal)
    pub4 = rospy.Publisher("/controller/cmd_head_ver", Float32, queue_size=1) #Camera (Vertical)
    us = uSound()
    sleep(0.5)
    rate = rospy.Rate(2)
    left = False
    right = False

    while 1:
        print("Distancias: E:%.2f cm; C:%.2f cm; D:%.2f cm" %(us.ranges[0],us.ranges[1],us.ranges[2]))
        if us.ranges[1]>80:
            goForward(us, pub1, 1)
        else:
            goForward(us, pub1, 0)
            sleep(1)
            right = check(us, pub2, 90)
            sleep(0.4)
            sleep(1)
            right = check(us, pub2, 180)
            sleep(0.4)
            pub2.publish(90)
            sleep(1)
            if right and left:
                #Camera
                pub2.publish(90)
            else:
                if left:
                    pub2.publish(270)
                else:
                    pub2.publish(90)
            sleep(1)
            left = False
            right = False

        rate.sleep()
