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
import random 
from nubot_common.msg import OminiVisionInfo
from nubot_common.msg import VelCmd
from nubot_common.srv import *
from turnHead2Kick import *
from std_msgs.msg import Bool, Int16, String, Float32MultiArray
from std_srvs.srv import Empty
from strategy.msg import A_info
import math
import time
import numpy as np

_state = "null"
#adjustable parameter
angle_thres = 0.05 * 1 #(*1 a little bit slow)
RotConst = 6 #4 maybe 6
KickPwr = 2 #2
MaxSpd_A = 250 #無關 200 or 250
MaxSpd_B = 150 #無關 200 or 250

class Strategy(object):
    def __init__(self, team):
        self.team = team
        self.RadHead2Ball = 0.0
        self.RadHead = 0.0
        self.NunbotAPosX = 0.0
        self.NunbotAPosY = 0.0
        self.BallPosX = 0.0
        self.BallPosY = 0.0
        self.GoalX = 900.0
        self.GoalY = 0.0
        self.StartX = 0.0
        self.StartY = 0.0
        self.kick_count = 0
        self.kick_num = 0 
        self.score = 0
        self.RadTurn = 0.0
        self.Steal = False 
        self.dis2start = 0.0
        self.dis2goal = 0.0
        self.vec = VelCmd()
        self.A_info = [0]
        self.game_count = 2
    def callback(self, data): # Rostopic 之從外部得到的值
        self.RadHead2Ball = data.ballinfo.real_pos.angle 
        self.RadHead = data.robotinfo[0].heading.theta
        self.BallPosX = data.ballinfo.pos.x
        self.BallPosY = data.ballinfo.pos.y
        self.NunbotAPosX = data.robotinfo[0].pos.x
        self.NunbotAPosY = data.robotinfo[0].pos.y
    def steal_callback(self, data):
        self.Steal = data.data
    # def kick_callback(self, data):
    #     self.kick_num = data.data
    # def dis2goal_callback(self, data):
    #     self.dis2goal = data.data
    # def dis2start_callback(self, data):
    #     self.dis2start = data.data
    def A_info_callback(self, data):
        self.A_info = data.data
    def state_callback(self,data):
        self.kick_count = 0
    def ros_init(self):
        if self.team == 'A':
            rospy.init_node('strategy_node_A', anonymous=True)
            #初始化一個節點,命名爲'strategy_node'
            #Publisher
            # self.kick_pub = rospy.Publisher('/nubot1/kick', Int16, queue_size=10)
            # self.dis2goal_pub = rospy.Publisher('/nubot1/dis2goal', Float64MultiArray, queue_size=10)
            # self.dis2start_pub = rospy.Publisher('/nubot1/dis2start', Float64MultiArray, queue_size=10)
            self.A_info_pub = rospy.Publisher('/nubot1/A_info', Float32MultiArray, queue_size=1) # 3in1
            self.vel_pub = rospy.Publisher('/nubot1/nubotcontrol/velcmd', VelCmd, queue_size=10)
            rospy.Subscriber("/nubot1/omnivision/OmniVisionInfo", OminiVisionInfo, self.callback)
            rospy.Subscriber('/coach/state', String, self.state_callback)
            #Subscriber with call back
            rospy.wait_for_service('/nubot1/BallHandle')
            self.call_Handle = rospy.ServiceProxy('/nubot1/BallHandle', BallHandle)
            rospy.wait_for_service('/nubot1/Shoot')
            self.call_Shoot = rospy.ServiceProxy('/nubot1/Shoot', Shoot)
            rospy.wait_for_service('/nubot1/Shoot')
        elif self.team == 'B':
            rospy.init_node('strategy_node_B', anonymous=True)
            self.vel_pub = rospy.Publisher('/rival1/nubotcontrol/velcmd', VelCmd, queue_size=10)
            self.steal_pub = rospy.Publisher('/rival1/steal', Bool, queue_size=1) # steal
            rospy.Subscriber("/rival1/omnivision/OmniVisionInfo", OminiVisionInfo, self.callback)
            #Subscriber with call back
            rospy.wait_for_service('/rival1/BallHandle')
            self.call_Handle = rospy.ServiceProxy('/rival1/BallHandle', BallHandle)
            rospy.wait_for_service('/rival1/Shoot')
            self.call_Shoot = rospy.ServiceProxy('/rival1/Shoot', Shoot)
        else :
            rospy.init_node('coach', anonymous=True)
            self.state_pub = rospy.Publisher('/coach/state', String, queue_size=10)
            rospy.Subscriber("/nubot1/omnivision/OmniVisionInfo", OminiVisionInfo, self.callback)
            rospy.Subscriber("/rival1/steal", Bool, self.steal_callback) # steal
            # rospy.Subscriber("/nubot1/kick", Int16, self.kick_callback)
            # rospy.Subscriber("/nubot1/dis2goal", Float64MultiArray, self.dis2goal_callback)
            # rospy.Subscriber("/nubot1/dis2start", Float64MultiArray, self.dis2start_callback)
            rospy.Subscriber("/nubot1/A_info", Float32MultiArray, self.A_info_callback)
            # self.vel_pub = rospy.Publisher('/nubot1/nubotcontrol/velcmd', VelCmd, queue_size=10)
            # self.call_Handle = rospy.ServiceProxy('/nubot1/BallHandle', BallHandle)
            # self.call_Shoot = rospy.ServiceProxy('/rival1/Shoot', Shoot)
            rospy.wait_for_service('/gazebo/reset_world')
            self.call_restart = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        # service
        # rate = rospy.Rate(1000000000000000000000000)#10
    def ball_out(self):
        if self.BallPosX >= 875 or self.BallPosX <= -875 or self.BallPosY >= 590 or self.BallPosY <= -590 :
            self.show('Out')
            return True
        else:
            return False
    def steal(self):
        if self.Steal:
            self.show('steal')
        return self.Steal
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
        # self.kick_pub.publish(self.kick_count)
        # A = A_info()
        # A.list = [self.kick_count, self.cal_dis2goal(), self.cal_dis2start()]
        data = Float32MultiArray()
        data.data = [self.kick_count, self.cal_dis2goal(), self.cal_dis2start()]
        self.A_info_pub.publish(data)
        # self.cal_dis2goal()
        # self.cal_dis2start()
        time.sleep(0.05)
        print ("Kick: %d" %self.kick_count)

    def chase(self, MaxSpd):
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

    def cal_dis2goal(self): # last kick
        dis2goal_x = self.NunbotAPosX - self.GoalX
        dis2goal_y = self.NunbotAPosY - self.GoalY
        dis2goal = math.hypot(dis2goal_x, dis2goal_y)
        return dis2goal
        # self.dis2goal_pub.publish(dis2goal)

    def cal_dis2start(self): # last kick
        dis2start_x = self.NunbotAPosX - self.StartX
        dis2start_y = self.NunbotAPosY - self.StartY
        dis2start = math.hypot(dis2start_x, dis2start_y)
        return dis2start
        # self.dis2start_pub.publish(dis2start)

    def restart(self):
        game_state = "game is over"
        self.state_pub.publish(game_state)
        self.Steal = False
        # self.show('restart')
        print('Game %d over' %(self.game_count-1))
        self.show('---Restart---')
        print('Game %d start' %self.game_count)
        self.game_count += 1 
        self.call_restart()
    def workA(self):
        count = 1
        while not rospy.is_shutdown():
        # while not self.ball_out():
        # ball_out()
            res = self.call_Handle(1) #start holding device
            # print(turn(100))
            # print(res)
            if not self.call_Handle(1).BallIsHolding:  # BallIsHolding = 0
                self.call_Handle(1)
                self.chase(MaxSpd_A)
                
                rel_turn_ang = 45
                rel_turn_rad = math.radians(rel_turn_ang)
                self.RadTurn = rel_turn_rad + self.RadHead
                
            else: # BallIsHolding = 1
                global RadHead
                # [] you will recieve a relative kick ang (-180~180) rel_turn_ang
                # print(rel_turn_rad)
                # print(self.RadTurn)
                error = math.fabs (turnHead2Kick(self.RadHead, self.RadTurn))

                if error > angle_thres : # 還沒轉到    
                    self.turn(self.RadTurn)
                else : # 轉到
                    self.kick()

                    # angle4 = [ 0, 1.57, -1.57,3.14 ]
                    # self.RadTurn = angle4[count%4]
                    # count += 1             
    def workB(self):
        catch = False
        while not rospy.is_shutdown():
            self.call_Handle(1) #start holding device
            if not self.call_Handle(1).BallIsHolding:  # BallIsHolding = 0
                self.call_Handle(1)
                self.chase(MaxSpd_B)
                catch = False
            else: # BallIsHolding = 1
                self.chase(MaxSpd_B)
                if not catch:
                    catch = True
                    ticks = time.time()
                    ticks = ticks + 5 #sec
                if time.time() > ticks:
                    self.steal_pub.publish(True)
                    self.show('steal')
                    ticks += 5
    def workC(self):
        print('Game 1 start')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            '''
            '''
            # print()
            '''
            '''
            is_steal = self.steal()
            is_out = self.ball_out()
            if  is_steal or is_out:
                print(self.A_info)#3in1

                # []wait for computing then restart
                self.restart() # 3in1 # print(self.kick_num)# print(self.dis2goal)# print(self.dis2start)


            # data_pub()
            rate.sleep()

    def get_score(self):
        return self.score
    
# if __name__ == '__main__':
    # s = Strategy('A')
    # print(s.NunbotAPosX)
    # strategy1 = Strategy('A')
    # strategy1.ros_init()
    # strategy1.work()
    # strategy2 = Strategy('B')
    # strategy2.ros_init()
    # strategy2.work()