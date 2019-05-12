#!/usr/bin/env python3
# coding=UTF-8
# This Python file uses the following encoding: utf-8

# Subscriber 
#---How to write "Subscribe"---
# 1. When you want to Subscribe sth. from somewhere ( when u want to get info from somewhere ): 
#   $ rostopic echo /rival1/omnivision/OmniVisionInfo
#   Check how it echo out 
# Then get its 路徑 and Type（形態）
#   $ rostopic info /rival1/omnivision/OmniVisionInfo 
#   -> Type: nubot_common/OminiVisionInfo
# “data" in here means all data 
#   Can use the way of structor to get more detail data
#   e.g. print(data.ballinfo.real_pos)
# In callback fuc below 
#   e.g. rospy.Subscriber("/rival1/omnivision/OmniVisionInfo", OminiVisionInfo, callback)
#   =    rospy.Subscriber(" Ros的路徑 ", 形態 , callback)
# ---
import sys
import rospy
import random 
from nubot_common.msg import OminiVisionInfo
from nubot_common.msg import VelCmd
from nubot_common.srv import *
from turnHead2Kick import *
# from opponent import workOpp
from restart import restart
import math
import time

_state = "null"
#adjustable parameter
angle_thres = 0.05 * 1.5 #(*1 a little bit slow)
RotConst = 6 #4 maybe 6
KickPwr = 2 #2
MaxSpd = 150 #無關 200 or 250

class STRATEGY(object):
    def __init__(self):
        self.RadHead2Ball = 0.0
        self.RadHead = 0.0
        self.BallPosX = 0.0
        self.BallPosY = 0.0
        self.kick_count = 0
        self.vec = VelCmd()
    def callback(self, data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # print(data.ballinfo.real_pos)
    # global R 
    # R = data.ballinfo.real_pos.radius 
    # global RadHead2Ball # angle to ball
        self.RadHead2Ball = data.ballinfo.real_pos.angle 
        self.RadHead = data.robotinfo[0].heading.theta
        self.BallPosX = data.ballinfo.pos.x
        self.BallPosY = data.ballinfo.pos.y
    def ros_init(self):
        rospy.init_node('strategy_node', anonymous=True)
        #初始化一個節點,命名爲'strategy_node'
        #Publisher
        self.vel_pub = rospy.Publisher('/rival1/nubotcontrol/velcmd', VelCmd, queue_size=10)
        rospy.Subscriber("/rival1/omnivision/OmniVisionInfo", OminiVisionInfo, self.callback)
        #Subscriber with call back
        rospy.wait_for_service('/rival1/BallHandle')
        self.call_Handle = rospy.ServiceProxy('/rival1/BallHandle', BallHandle)
        rospy.wait_for_service('/rival1/Shoot')
        self.call_Shoot = rospy.ServiceProxy('/rival1/Shoot', Shoot)
        # service
        # rate = rospy.Rate(1000000000000000000000000)#10
        # resp1 = call_Shoot(1, 1) #cause 2 input in this Shoot case
    def ball_out(self):
        if self.BallPosX >= 895 or self.BallPosX <= -895 or self.BallPosY >= 595 or self.BallPosY <= -595 :
            self.show('Out')
            return True
        else:
            return False

    def show(self, state):
        global _state
        if  state != _state :
            print(state)
        _state = state

    def kick(self):
        self.vec.Vx = 0
        self.vec.Vy = 0
        self.vec.w = 0
        self.vel_pub.publish(self.vec)
        self.call_Shoot(KickPwr, 1) # power from GAFuzzy
        global kick_count
        self.kick_count = self.kick_count + 1
        time.sleep(0.01)
        print ("Kick: %d" %self.kick_count)

    def chase(self):
        self.vec.Vx = MaxSpd * math.cos(self.RadHead2Ball)
        self.vec.Vy = MaxSpd * math.sin(self.RadHead2Ball)
        self.vec.w = self.RadHead2Ball * RotConst
        self.vel_pub.publish(self.vec)
        self.show("Chasing")
        
    def turn(self, angle):
        self.vec.Vx = 0
        self.vec.Vy = 0
        self.vec.w = turnHead2Kick(self.RadHead, angle) * RotConst
        self.vel_pub.publish(self.vec)
        self.show("Turning")

    def work(self):
        catch = False
        while not rospy.is_shutdown():
            while not self.ball_out():
                # ball_out()
                self.call_Handle(1) #start holding device
                # print(turn(100))
                # print(res)
                if not self.call_Handle(1).BallIsHolding:  # BallIsHolding = 0
                    self.call_Handle(1)
                    self.chase()
                    catch = False
                else: # BallIsHolding = 1
                    self.chase()
                    if not catch:
                        catch = True
                        ticks = time.time()
                        ticks = ticks + 3 #sec
                    if time.time() > ticks:
                        print("game off")
if __name__ == '__main__':
    strategy = STRATEGY()
    strategy.ros_init()
    strategy.work()