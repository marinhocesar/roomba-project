#!/usr/bin/env python3

from time import sleep
from gpiozero import DistanceSensor # Import GPIO Zero Library
import rospy
from sensor_msgs.msg import LaserScan
# Define GPIO pins to use on the Pi

if __name__ == "__main__":

    pintrigger = 17
    pinecho = 18
    sensor = DistanceSensor(echo=pinecho, trigger=pintrigger)
    rospy.init_node("ultrasound_reader")
    l = LaserScan()
    l.ranges = [0,0,0]
    pub = rospy.Publisher("/ultrasound_reader/distance", LaserScan, queue_size=1)
    print("Ultrasonic Measurement")

    while True:
        l.ranges[1] = sensor.distance * 100
        print("Distance: %.1f cm" % (sensor.distance * 100))
        pub.publish(l)
        sleep(0.2)
