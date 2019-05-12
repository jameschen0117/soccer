#!/usr/bin/env python3

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
import math
from turnHead import *

R = 0.0
A = 0.0

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #print(data.ballinfo.real_pos)
    global R 
    R = data.ballinfo.real_pos.radius 
    global A
    A = data.ballinfo.real_pos.angle 

    #print(R)
    #print(A)
        
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('tmp_node', anonymous=True)
    #初始化一個節點,命名爲'listener'
    vel_pub = rospy.Publisher('/nubot1/nubotcontrol/velcmd', VelCmd, queue_size=10)
    rospy.Subscriber("/nubot1/omnivision/OmniVisionInfo", OminiVisionInfo, callback)
    rospy.wait_for_service('/nubot1/BallHandle')
    call_Handle = rospy.ServiceProxy('/nubot1/BallHandle', BallHandle)
    rospy.wait_for_service('/nubot1/Shoot')
    call_Shoot = rospy.ServiceProxy('/nubot1/Shoot', Shoot)
    rate = rospy.Rate(10)

    # resp1 = call_Shoot(1, 1) #cause 2 input in this Shoot case

    while not rospy.is_shutdown():
        res = call_Handle(1)
        print(res)
        if not call_Handle(1).BallIsHolding:  
            vec = VelCmd()
            vec.Vx = 200 * math.cos(A)
            vec.Vy = 200 * math.sin(A)
            vec.w = A
            vel_pub.publish(vec)
        else:
            # vec.w = #轉成踢球方向
            call_Shoot(50, 1) #from GAFuzzy


        #rospy.spin()
        rate.sleep()
    #Subscribe from "..." this topic

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    # listener()
    test()