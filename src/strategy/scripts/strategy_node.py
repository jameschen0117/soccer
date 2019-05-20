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
from sac import SAC
from nubot_common.msg import OminiVisionInfo
from nubot_common.msg import VelCmd
from nubot_common.srv import *
from turnHead2Kick import *
from std_msgs.msg import Bool, Int16, String, Float32MultiArray, Float32
from std_srvs.srv import Empty
# from strategy.msg import A_info
from sac_calculate import sac_calculate
from gazebo_msgs.msg import ModelStates
import math
import time
import numpy as np
 
pwr = 1.0

_state = "null"
#adjustable parameter
angle_thres = 0.05 * 1 #(*1 a little bit slow)
RotConst = 4 #4 maybe 6
MAX_PWR = 3 #2 or 3
MaxSpd_A = 150 #無關 200 or 250
MaxSpd_B = 30 #無關 200 or 250
l_rate = 1.0 # times(*)

class Strategy(object):
    def __init__(self, team):
        self.sac_cal = sac_calculate()

        self.a = []
        self.s = []
        self.r = 0.0
        self.done = False

        self.avg_arr = np.zeros(64)
        self.team = team
        self.RadHead2Ball = 0.0
        self.RadHead = 0.0
        self.NunbotAPosX = 0.0
        self.NunbotAPosY = 0.0
        self.BallPosX = 0.0
        self.BallPosY = 0.0
        self.GoalX = 900.0
        self.GoalY = 0.0
        self.StartX = -900.0
        self.StartY = 0.0
        self.kick_count = 0
        self.kick_num = 0 
        self.score = 0
        self.RadTurn = 0.0
        self.Steal = False 
        self.dis2start = 0.0
        self.dis2goal = 0.0
        self.vec = VelCmd()
        self.A_info = np.array([1.0, 1.0, 1.0, 1.0, 0, 0, 0, 0])
        self.game_count = 2
        self.A_z = 0.0
        self.B_z = 0.0
        self.HowEnd = 0
        self.B_dis = 0.0
        self.is_kick = False
        self.ready2restart = True
    def callback(self, data): # Rostopic 之從外部得到的值
        self.RadHead2Ball = data.ballinfo.real_pos.angle 
        self.RadHead = data.robotinfo[0].heading.theta
        self.BallPosX = data.ballinfo.pos.x
        self.BallPosY = data.ballinfo.pos.y
        self.NunbotAPosX = data.robotinfo[0].pos.x
        self.NunbotAPosY = data.robotinfo[0].pos.y
        self.B_dis = data.obstacleinfo.polar_pos[0].radius
    def steal_callback(self, data):
        self.Steal = data.data
    def A_info_callback(self, data):
        self.A_info = np.array(data.data)
        self.is_kick = True

    def state_callback(self,data):
        self.kick_count = 0
    def reward_callback(self, data):
        pass
        # self.r = data.data
    def done_callback(self, data):
        self.done = data.data
    def fly_callback(self, data):
        self.A_z = data.pose[5].position.z
        self.B_z = data.pose[6].position.z
    def HowEnd_callback(self,data):
        self.HowEnd = data.data
    def ready2restart_callback(self, data):
        self.restart()
        self.ready2restart = False

    def ros_init(self):
        if self.team == 'A':
            self.agent = SAC(act_dim=2, obs_dim=11,
            lr_actor=l_rate*(1e-3), lr_value=l_rate*(1e-3), gamma=0.99, tau=0.995)
    
            rospy.init_node('strategy_node_A', anonymous=True)
            # self.A_info_pub = rospy.Publisher('/nubot1/A_info', Float32MultiArray, queue_size=1) # 3in1
            self.vel_pub    = rospy.Publisher('/nubot1/nubotcontrol/velcmd', VelCmd, queue_size=1)
            # self.ready2restart_pub  = rospy.Publisher('nubot1/ready2restart',Bool, queue_size=1)
            rospy.Subscriber("/nubot1/omnivision/OmniVisionInfo", OminiVisionInfo, self.callback)
            # rospy.Subscriber('/coach/state', String, self.state_callback)
            # rospy.Subscriber('/coach/reward', Float32, self.reward_callback)
            # rospy.Subscriber('/coach/done', Bool, self.done_callback)
            # rospy.Subscriber('coach/HowEnd', Int16, self.HowEnd_callback)
            # rospy.Subscriber("/rival1/steal", Bool, self.steal_callback)
            
            rospy.wait_for_service('/nubot1/BallHandle')
            self.call_Handle = rospy.ServiceProxy('/nubot1/BallHandle', BallHandle)
            
            rospy.wait_for_service('/nubot1/Shoot')
            self.call_Shoot = rospy.ServiceProxy('/nubot1/Shoot', Shoot)
            
            rospy.wait_for_service('/gazebo/reset_world')
            self.call_restart = rospy.ServiceProxy('/gazebo/reset_world', Empty, persistent=True)

            rospy.wait_for_service('/rival1/BallHandle')
            self.call_B_Handle = rospy.ServiceProxy('/rival1/BallHandle', BallHandle)
        elif self.team == 'B':
            rospy.init_node('strategy_node_B', anonymous=True)
            self.vel_pub   = rospy.Publisher('/rival1/nubotcontrol/velcmd', VelCmd, queue_size=1)
            self.steal_pub = rospy.Publisher('/rival1/steal', Bool, queue_size=1) # steal
            rospy.Subscriber("/rival1/omnivision/OmniVisionInfo", OminiVisionInfo, self.callback)
            rospy.wait_for_service('/rival1/BallHandle')
            self.call_Handle = rospy.ServiceProxy('/rival1/BallHandle', BallHandle)
            rospy.wait_for_service('/rival1/Shoot')
            self.call_Shoot = rospy.ServiceProxy('/rival1/Shoot', Shoot)
        else :
            rospy.init_node('coach', anonymous=True)
            self.state_pub  = rospy.Publisher('/coach/state', String, queue_size=1)
            self.reward_pub = rospy.Publisher('/coach/reward', Float32, queue_size=1)
            self.done_pub   = rospy.Publisher('coach/done', Bool, queue_size=1)
            self.HowEnd_pub = rospy.Publisher('coach/HowEnd', Int16, queue_size=1)
            rospy.Subscriber("/nubot1/omnivision/OmniVisionInfo", OminiVisionInfo, self.callback)
            rospy.Subscriber("/rival1/steal", Bool, self.steal_callback) # steal
            rospy.Subscriber("/nubot1/A_info", Float32MultiArray, self.A_info_callback)
            rospy.Subscriber('gazebo/model_states', ModelStates, self.fly_callback)
            rospy.Subscriber('nubot1/ready2restart',Bool , self.ready2restart_callback)
            rospy.wait_for_service('/gazebo/reset_world')
            self.call_restart = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    def ball_out(self):
        if self.BallPosX >= 875 or self.BallPosX <= -875 or self.BallPosY >= 590 or self.BallPosY <= -590 :
            self.show('Out')
            return True
        else:
            return False

    def ball_in(self):
        if self.BallPosX >= 670 and self.BallPosX <= 875 and self.BallPosY >= -330 and self.BallPosY <= 330 :
            self.show('in')
            return True
        else:
            return False
    def fly(self):
        if self.A_z > 0.2 or self.B_z > 0.2 :
            return True
        else:
            return False
    def steal(self):
        if self.call_B_Handle(1).BallIsHolding:
            return True
        else:
            return False
        

    def stealorfly(self):
        if self.steal() or self.fly():
            if self.steal():
                self.show('steal')
            else:
                self.show('fly')
            return True
        else:
            return False

    def show(self, state):
        global _state
        if  state != _state :
            print(state)
        _state = state

    def cnt_rwd(self):
        data = Float32MultiArray()
        data.data = [self.kick_count, self.cal_dis2start(), self.cal_dis2goal(), self.B_dis, 0, 0, 0, 0]
        if self.game_is_done():
            if self.HowEnd == 1:
                data.data[4] = 1
                data.data[5] = 0
                data.data[6] = 0
            if self.HowEnd == -1:
                data.data[4] = 0
                data.data[5] = 0
                data.data[6] = 1
            if self.HowEnd == -2:
                data.data[4] = 0
                data.data[5] = 1
                data.data[6] = 0
        # self.A_info_pub.publish(data) # 2C
        self.sac_cal = sac_calculate()
        reward = self.sac_cal.reward(data.data)
        data.data[7] = reward
        print('rwd init',['kck_n  g_dis st_dis  opp_dis  in    out   steal   ttl'])
        print('rwd unit',np.around((data.data), decimals=1 ))
        print('rwd :',reward)
        return(reward)
        
    def kick(self):
        self.vec.Vx = 0
        self.vec.Vy = 0
        self.vec.w = 0
        self.vel_pub.publish(self.vec)
        global pwr
        self.call_Shoot(pwr, 1) # power from GAFuzzy
        global kick_count
        self.kick_count = self.kick_count + 1
        time.sleep(0.2)
        print ("Kick: %d" %self.kick_count)
        print('---')
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

    def cal_dis2start(self): # last kick
        dis2start_x = self.NunbotAPosX - self.StartX
        dis2start_y = self.NunbotAPosY - self.StartY
        dis2start = math.hypot(dis2start_x, dis2start_y)
        return dis2start
        # self.dis2start_pub.publish(dis2start)

    def cal_dis2goal(self): # last kick
        dis2goal_x = self.NunbotAPosX - self.GoalX
        dis2goal_y = self.NunbotAPosY - self.GoalY
        dis2goal = math.hypot(dis2goal_x, dis2goal_y)
        return dis2goal
        # self.dis2goal_pub.publish(dis2goal)

    def avg(self, n, l):
        l = np.delete(l, 0)
        l = np.append(l, n)
        self.avg_arr = l
        print (self.avg_arr)
        print (sum(l)/64)  

    def restart(self):
        # game_state_word = "game is over"
        # self.state_pub.publish(game_state_word) # 2A
        # self.Steal = False
        print('Game %d over' %(self.game_count-1))
        self.show('---Restart---')
        print('Game %d start' %self.game_count)
        self.game_count += 1
        self.kick_count = 0
        print('i want to go in rospy.wait_for_service(/gazebo/reset_world)')
        rospy.wait_for_service('/gazebo/reset_world')
        print('i want to self.call_restart()')
        self.call_restart()
        print('after call_restart')
        self.ready2restart =False
        print('i finish def restart(self)')
        

    def coach(self):
        pass
    def game_is_done(self):
        if self.ball_in() or self.ball_out() or self.stealorfly():
            if self.ball_in():
                self.HowEnd = 1
            if self.ball_out():
                self.HowEnd = -2
            if self.stealorfly():
                self.HowEnd = -1
            return True
        else:
            return False



    def workA(self):
        np.set_printoptions(suppress=True)
        i = 0
        fisrt_time_hold = False
        real_resart = True
        while not rospy.is_shutdown():
            # print(self.ball_in(), self.ball_out(), self.stealorfly())
            self.call_Handle(1) # open holding device

            if self.game_is_done() and real_resart:
                self.r = self.cnt_rwd()
                # print('h',self.HowEnd)
                s_ = self.sac_cal.input(self.HowEnd) #out state
                if i > 1:
                    if len(self.s) == 11 and len(s_) == 11:
                        print('000000000000000000',np.shape(self.s), np.shape(self.a))
                        self.agent.replay_buffer.store_transition(self.s , self.a, self.r, s_, self.done)
                    # print('d',self.done)
                    print('ep rwd value=',self.r)
                i += 1 
                self.s = s_
                self.done = False
                if i > 64:
                    self.agent.learn(i, self.r)
                # self.ready2restart_pub.publish(True)
                # self.ready2restart_pub.publish(False)
                real_resart = False
                self.HowEnd=0
                print('i want to go in self.restart()')
                self.restart()
                # print('---')
            elif not self.game_is_done():
                if not self.call_Handle(1).BallIsHolding :  # BallIsHolding = 0
                    self.chase(MaxSpd_A)
                    fisrt_time_hold = True
                    real_resart = True
                elif self.call_Handle(1).BallIsHolding : # BallIsHolding = 1
                    global RadHead
                    if fisrt_time_hold == True:
                        self.show('Touch')
                        self.r = self.cnt_rwd()
                        s_ = self.sac_cal.input(0)                 #state_for_sac
                        if i >= 1:
                            if len(self.s) == 11 and len(s_) == 11:
                                # print('11111111111111111',np.shape(self.s), np.shape(self.a))
                                self.agent.replay_buffer.store_transition(self.s, self.a, self.r, s_, self.done)
                            print('step rwd value= ',self.r)
                        self.done = False
                        i += 1  
                        self.s = s_
                        self.a = self.agent.choose_action(self.s)        #action_from_sac
                        rel_turn_ang = self.sac_cal.output(self.a)       #action_from_sac

                        global pwr, MAX_PWR
                        pwr = (self.a[1]+1) * MAX_PWR/2 + 0.5    #normalize
                        # sac]
                                        
                        rel_turn_rad = math.radians(rel_turn_ang)
                        self.RadTurn = rel_turn_rad + self.RadHead
                        fisrt_time_hold = False
                        if i > 64:
                            self.agent.learn(i, self.r)
                    elif fisrt_time_hold == False:
                        error = math.fabs (turnHead2Kick(self.RadHead, self.RadTurn))
                        if error > angle_thres : # 還沒轉到    
                            self.turn(self.RadTurn)
                        else : # 轉到
                            self.kick()
                            
    def workB(self):
        # catch = False
        while not rospy.is_shutdown():
            self.call_Handle(1) #start holding device
            if not self.call_Handle(1).BallIsHolding:  # BallIsHolding = 0
                # self.call_Handle(1)
                self.chase(MaxSpd_B)
                # catch = False
            else: # BallIsHolding = 1
                # self.chase(MaxSpd_B)
                # if not catch:
                #     catch = True
                #     ticks = time.time()
                #     ticks = ticks + 3 # sec # steal time
                # if time.time() > ticks:
                # self.steal_pub.publish(True) # 2C
                self.show('steal')
                # ticks += 5
    
    def coach(self):
        pass

    def workC(self):
        print('Game 1 start')
        # rate = rospy.Rate(10)
        np.set_printoptions(suppress=True)
        while not rospy.is_shutdown():

            is_stealorfly = self.stealorfly() 
            is_out = self.ball_out()
            is_in = self.ball_in()

            # # [] send rwd 2 A  
            
            # self.sac_cal = sac_calculate()
            # # self.A_info = list(self.A_info)

            # # if self.is_kick:
            # self.A_info[4] = 0
            # self.A_info[5] = 0
            # self.A_info[6] = 0
            
            # reward = self.sac_cal.reward(self.A_info)   # rwd 2 A
            # self.reward_pub.publish(reward) 
            # print('step rwd unit = ', np.around((self.A_info), decimals=1 )) # 7in1 # 7 rwd unit
            # print('step rwd value =',reward)
            #     # self.is_kick = False

            if  is_in or is_out or is_stealorfly:
                # done 2 A
                # self.ready2restart = False
                if is_in:
                    HowEnd = 1
                #     self.A_info[4] = 1
                #     self.A_info[5] = 0
                #     self.A_info[6] = 0
                if is_stealorfly: 
                    HowEnd = -1
                #     self.A_info[4] = 0
                #     self.A_info[5] = 0
                #     self.A_info[6] = 1
                if is_out :
                    HowEnd = -2
                #     self.A_info[4] = 0
                #     self.A_info[5] = 1
                #     self.A_info[6] = 0
                self.HowEnd_pub.publish(HowEnd)
                self.done_pub.publish(True) 
                if self.ready2restart:
                    self.restart()
                
                # print('rwd unit = ', np.around((self.A_info), decimals=1 )) # 7in1 # 7 rwd unit

                # reward = self.sac_cal.reward(self.A_info)   # rwd 2 A
                # self.reward_pub.publish(reward)             # rwd 2 A
                 # HowEnd 2 A
                
                 

                # print('rwd value =',reward)
                
                # self.avg(reward, self.avg_arr)
                # []wait for computing then restart
                
                
                # while not self.ready2restart
                #     time.sleep(0.01)
                # self.restart()
                # time.sleep(0.5) 
                # 3in1 # print(self.kick_num)# print(self.dis2goal)# print(self.dis2start)
            # rate.sleep()

    # def get_score(self):
    #     return self.score
    
if __name__ == '__main__':
    s = Strategy('C')
    s.ros_init()
    while not rospy.is_shutdown():
        print(s.fly())