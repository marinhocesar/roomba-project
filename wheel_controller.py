#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from gpiozero import CamJamKitRobot
from time import sleep

class wheel:
    def __init__(self):
        self.robot = CamJamKitRobot()
        self.sub1 = rospy.Subscriber('/controller/cmd_dis', Float32, self.Forward)
        self.sub2 = rospy.Subscriber('/controller/cmd_ang', Float32, self.Rotate)

    def Forward(self,msg): #veiculo se desloca para frente
        if(msg.data==0):
            self.robot.stop()
        else:
            duty_cycle = 0.8 # 0 -> 0%, 1 -> 100%
            forward = (duty_cycle,duty_cycle) #Left and Right are inverteds
            self.robot.value = forward
            sleep((msg.data)*(1.5))
        # forward = (0.1,0.1)
        # self.robot.value = forward
        # sleep(0.2)

    def Rotate(self,msg): #veiculo gira em torno do eixo z
        duty_cycle = 0.24 # 0 -> 0%, 1 -> 100%
        if msg.data < 0:
            r = (-duty_cycle, duty_cycle) #sentido anti-horario
        else:
            r = (duty_cycle, -(duty_cycle)) #sentido horario
        self.robot.value = r
        sleep((abs((msg.data))/180)*1.1)
        self.robot.stop()


if __name__ == "__main__":

    rospy.init_node("wheel_controller")
    w = wheel()
    rospy.spin()
