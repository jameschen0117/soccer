#!/usr/bin/env python3
# coding=UTF-8
# This Python file uses the following encoding: utf-8

# Subscriber 
#---How to write "Subscribe"---
# 1. When you want to Subscribe sth. from somewhere ( when u want to get info from somewhere ): 
#   $ rostopic echo /nubot1/omnivision/OmniVisionInfo
#   Check how it echo out 
# Then get its 路徑 and Type（形態）
#   $ rostopic info /nubot1/omnivision/OmniVisionInfo 
#   -> Type: nubot_common/OminiVisionInfo
# “data" in here means all data 
#   Can use the way of structor to get more detail data
#   e.g. print(data.ballinfo.real_pos)
# In callback fuc below 
#   e.g. rospy.Subscriber("/nubot1/omnivision/OmniVisionInfo", OminiVisionInfo, callback)
#   =    rospy.Subscriber(" Ros的路徑 ", 形態 , callback)
# ---
import sys
import rospy
from nubot_common.msg import OminiVisionInfo
from nubot_common.msg import VelCmd
from nubot_common.srv import *
from turnHead import *
import math

global a
global R 
R = 0.0
global A2B 
A2B = 0.0
y1 = 3.14 # from GAFuzzy

MaxSpd = 200

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #print(data.ballinfo.real_pos)
    # global R 
    R = data.ballinfo.real_pos.radius 
    # global A2B # angle to ball
    A2B = data.ballinfo.real_pos.angle 

    #print(R)
    #print(A)
        
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('strategy_node', anonymous=True)
    #初始化一個節點,命名爲'strategy_node'
    vel_pub = rospy.Publisher('/nubot1/nubotcontrol/velcmd', VelCmd, queue_size=10)
    #Publisher
    rospy.Subscriber("/nubot1/omnivision/OmniVisionInfo", OminiVisionInfo, callback)
    #Subscriber with call back
    #
    rospy.wait_for_service('/nubot1/BallHandle')
    call_Handle = rospy.ServiceProxy('/nubot1/BallHandle', BallHandle)
    #
    rospy.wait_for_service('/nubot1/Shoot')
    call_Shoot = rospy.ServiceProxy('/nubot1/Shoot', Shoot)
    # service
    rate = rospy.Rate(10)

    # resp1 = call_Shoot(1, 1) #cause 2 input in this Shoot case

    while not rospy.is_shutdown():
        res = call_Handle(1) #start holding device
        print(turn(100))
        #print(res)
        if not call_Handle(1).BallIsHolding:  # BallIsHolding = 0
            vec = VelCmd()
            vec.Vx = MaxSpd * math.cos(A2B)
            vec.Vy = MaxSpd * math.sin(A2B)
            vec.w = A2B
            vel_pub.publish(vec)
        else: #when it holds ball
            # if 
            #     vec.w = 
            # y1 = # from GAFuzzy

        #    vec.w = vec.w  #  轉成踢球方向 from GAFuzzy
            call_Shoot(50, 1) # from GAFuzzy


        #rospy.spin()
        rate.sleep()
    #Subscribe from "..." this topic

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    listener()