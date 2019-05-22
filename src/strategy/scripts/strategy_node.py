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
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Twist
# from strategy.msg import A_info
from sac_calculate import sac_calculate
from gazebo_msgs.msg import ModelState, ModelStates
import math
import time
import numpy as np
 
pwr = 1.0

_state = "null"
#adjustable parameter
angle_thres = 0.05 * 1 #(*1 a little bit slow)
RotConst = 3 #4 maybe 6 # ? max = 3 
MAX_PWR = 2 #2 or 3
MaxSpd_A = 100 #無關 200 or 250
MaxSpd_B = 90 #無關 200 or 250
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
        self.ep_rwd = 0
        self.is_kick = False
        self.ready2restart = True
        self.list_rate = list(np.zeros(128))
        self.milestone=[0, 0, 0, 0, 0, 0, 0]
        self.milestone_idx =0 
        self.is_in = False
        self.is_out = False
        self.is_steal = False
        self.is_fly = False
        self.is_stealorfly = False
        self.real_resart = True
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
        self.A_z = data.pos[5].position.z
        self.B_z = data.pos[6].position.z
    def HowEnd_callback(self,data):
        self.HowEnd = data.data
    def ready2restart_callback(self, data):
        self.restart()
        self.ready2restart = False

    def ros_init(self):
        if self.team == 'A':
            self.agent = SAC(act_dim=2, obs_dim=12,
            lr_actor=l_rate*(1e-3), lr_value=l_rate*(1e-3), gamma=0.99, tau=0.995)
    
            rospy.init_node('strategy_node_A', anonymous=True)
            # self.A_info_pub = rospy.Publisher('/nubot1/A_info', Float32MultiArray, queue_size=1) # 3in1
            self.vel_pub    = rospy.Publisher('/nubot1/nubotcontrol/velcmd', VelCmd, queue_size=1)
            self.reset_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
            # self.ready2restart_pub  = rospy.Publisher('nubot1/ready2restart',Bool, queue_size=1)
            rospy.Subscriber("/nubot1/omnivision/OmniVisionInfo", OminiVisionInfo, self.callback)
            # rospy.Subscriber('/coach/state', String, self.state_callback)
            # rospy.Subscriber('/coach/reward', Float32, self.reward_callback)
            # rospy.Subscriber('/coach/done', Bool, self.done_callback)
            # rospy.Subscriber('coach/HowEnd', Int16, self.HowEnd_callback)
            # rospy.Subscriber("/rival1/steal", Bool, self.steal_callback)
            

            
            rospy.wait_for_service('/nubot1/Shoot')
            self.call_Shoot = rospy.ServiceProxy('/nubot1/Shoot', Shoot)
            
            # rospy.wait_for_service('/gazebo/reset_simulation')
            # self.call_restart = rospy.ServiceProxy('/gazebo/reset_simulation', Empty, persistent=True)

            # rospy.wait_for_service('/gazebo/set_model_state')
            # self.call_set_modol = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            rospy.wait_for_service('/nubot1/BallHandle')
            self.call_Handle = rospy.ServiceProxy('/nubot1/BallHandle', BallHandle)
            rospy.wait_for_service('/rival1/BallHandle')
            self.call_B_Handle = rospy.ServiceProxy('/rival1/BallHandle', BallHandle)
        elif self.team == 'B':
            rospy.init_node('strategy_node_B', anonymous=True)
            self.vel_pub   = rospy.Publisher('/rival1/nubotcontrol/velcmd', VelCmd, queue_size=1)
            self.steal_pub = rospy.Publisher('/rival1/steal', Bool, queue_size=1) # steal
            rospy.Subscriber("/rival1/omnivision/OmniVisionInfo", OminiVisionInfo, self.callback)
            rospy.wait_for_service('/rival1/BallHandle')
            self.call_Handle = rospy.ServiceProxy('/rival1/BallHandle', BallHandle)

        else :
            rospy.init_node('coach', anonymous=True)
            self.state_pub  = rospy.Publisher('/coach/state', String, queue_size=1)
            self.reward_pub = rospy.Publisher('/coach/reward', Float32, queue_size=1)
            self.done_pub   = rospy.Publisher('coach/done', Bool, queue_size=1)
            self.HowEnd_pub = rospy.Publisher('coach/HowEnd', Int16, queue_size=1)
            rospy.Subscriber("/nubot1/omnivision/OmniVisionInfo", OminiVisionInfo, self.callback)
            rospy.Subscriber("/rival1/steal", Bool, self.steal_callback) # steal
            rospy.Subscriber("/nubot1/A_info", Float32MultiArray, self.A_info_callback)
            # rospy.Subscriber('gazebo/model_states', ModelStates, self.fly_callback)
            rospy.Subscriber('nubot1/ready2restart',Bool , self.ready2restart_callback)
            rospy.wait_for_service('/gazebo/reset_simulation')
            self.call_restart = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    def ball_out(self):
        if self.BallPosX >= 875 or self.BallPosX <= -875 or self.BallPosY >= 590 or self.BallPosY <= -590 :
            self.show('Out')
            self.is_out = True
        
    def ball_in(self):
        if self.BallPosX >= 670 and self.BallPosX <= 875 and self.BallPosY >= -330 and self.BallPosY <= 330 :
            self.show('in')
            self.is_in = True

    def fly(self):
        if self.A_z > 0.2 or self.B_z > 0.2 :
            self.is_fly = True
    def steal(self):
        rospy.wait_for_service('/nubot1/BallHandle')
        rospy.wait_for_service('/rival1/BallHandle')
        if self.call_B_Handle(1).BallIsHolding and not self.call_Handle(1).BallIsHolding:
            self.is_steal = True

    def stealorfly(self):
        if self.is_fly or self.is_steal:
            self.is_stealorfly = True
        
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
                
            elif self.HowEnd == -1:
                data.data[4] = 0
                data.data[5] = 0
                data.data[6] = 1
                
            elif self.HowEnd == -2:
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
        global MaxSpd_A
        self.vec.Vx = MaxSpd_A * math.cos(self.RadHead2Ball)
        self.vec.Vy = MaxSpd_A * math.sin(self.RadHead2Ball)
        # self.vec.w = self.RadHead2Ball * RotConst
        self.vec.w = 0
        self.vel_pub.publish(self.vec)
        global pwr
        # rospy.wait_for_service('/nubot1/Shoot')
        self.call_Shoot(pwr, 1) # power from GAFuzzy
        while 1 :
            self.chase(MaxSpd_A)
            if self.game_is_done() and self.real_resart:
                break
            if not self.call_Handle(1).BallIsHolding:
                self.kick_count = self.kick_count + 1
                # time.sleep(0.2)
                print ("Kick: %d" %self.kick_count)
                print('-------')
                break
            print('in')
                
            
    def chase(self, MaxSpd):
        self.vec.Vx = MaxSpd * math.cos(self.RadHead2Ball)
        self.vec.Vy = MaxSpd * math.sin(self.RadHead2Ball)
        self.vec.w = self.RadHead2Ball * RotConst
        self.vel_pub.publish(self.vec)
    def chase_B(self, MaxSpd):
        self.vec.Vx = MaxSpd * math.cos(self.RadHead2Ball)
        self.vec.Vy = MaxSpd * math.sin(self.RadHead2Ball)
        self.vec.w = self.RadHead2Ball * RotConst/4
        self.vel_pub.publish(self.vec)
        # self.show("Chasing")
    def turn(self, angle):
        global MaxSpd_A
        self.vec.Vx = MaxSpd_A * math.cos(self.RadHead2Ball)
        self.vec.Vy = MaxSpd_A * math.sin(self.RadHead2Ball)
        # self.vec.Vx = 0
        # self.vec.Vy = 0
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

    def reset_ball(self):
        Ball_msg = ModelState()
        Ball_msg.model_name = 'football'
        Ball_msg.pose.position.x = -6 #-6.8
        Ball_msg.pose.position.y = 0
        Ball_msg.pose.position.z = 0.12
        Ball_msg.pose.orientation.x = 0
        Ball_msg.pose.orientation.y = 0
        Ball_msg.pose.orientation.z = 0
        Ball_msg.pose.orientation.w = 1
        self.reset_pub.publish(Ball_msg)
    def reset_A(self):
        A_msg = ModelState()
        A_msg.model_name = 'nubot1'
        A_msg.pose.position.x = -8 #-8.5
        A_msg.pose.position.y = 0
        A_msg.pose.position.z = 0
        A_msg.pose.orientation.x = 0
        A_msg.pose.orientation.y = 0
        A_msg.pose.orientation.z = 0
        A_msg.pose.orientation.w = 1
        self.reset_pub.publish(A_msg)
    def reset_B(self):
        B_msg = ModelState()
        B_msg.model_name = 'rival1'
        B_msg.pose.position.x = 2 #0
        B_msg.pose.position.y = 0
        B_msg.pose.position.z = 0
        B_msg.pose.orientation.x = 0
        B_msg.pose.orientation.y = 0
        B_msg.pose.orientation.z = 0
        B_msg.pose.orientation.w = 1
        self.reset_pub.publish(B_msg)

    def restart(self):
        # game_state_word = "game is over"
        # self.state_pub.publish(game_state_word) # 2A
        # self.Steal = False
        self.reset_ball() 
        self.reset_ball()
        print('Game %d over' %(self.game_count-1))
        print('-----------Restart-----------')
        print('Game %d start' %self.game_count)
        self.reset_A()
        self.reset_A() 
        self.game_count += 1
        self.kick_count = 0
        self.reset_B()
        self.reset_B() 
        # self.call_set_modol(SetModelState)

        # print('after call_restart')
        self.ready2restart =False
        self.is_fly = False
        self.is_steal = False
        self.is_stealorfly = False
        self.is_in = False
        self.is_out = False

        # print('i finish def restart(self)')
    def end_rate(self, end):
        self.list_rate[self.game_count%128] = end
        out_count = self.list_rate.count(-2)
        in_count  = self.list_rate.count(1)
        steal_count=self.list_rate.count(-1)
        print('in_rate',in_count/128,'out_rate',out_count/128,'steal_rate',steal_count/128)
        if in_count/128 != 0  and self.milestone_idx == 0:
            self.milestone[0]=self.game_count
            self.milestone_idx = self.milestone_idx +1 
        if in_count/128 >= 0.1 and self.milestone_idx ==1:
            self.milestone[1]=self.game_count
            self.milestone_idx = self.milestone_idx +1 
        if in_count/128 >= 0.2 and self.milestone_idx ==2:
            self.milestone[2]=self.game_count
            self.milestone_idx = self.milestone_idx +1 
        if in_count/128 >= 0.5 and self.milestone_idx ==3:
            self.milestone[3]=self.game_count
            self.milestone_idx = self.milestone_idx +1 
        if in_count/128 >= 0.8 and self.milestone_idx ==4:
            self.milestone[4]=self.game_count
            self.milestone_idx = self.milestone_idx +1
        if in_count/128 >= 0.9 and self.milestone_idx ==5:
            self.milestone[5]=self.game_count
            self.milestone_idx = self.milestone_idx +1
        if in_count/128 == 1  and self.milestone_idx ==6:
            self.milestone[6]=self.game_count
            self.milestone_idx = self.milestone_idx +1
        print('milestone',self.milestone)
    def game_is_done(self):
        self.ball_in()
        self.ball_out()
        self.steal()
        self.fly()
        self.stealorfly()
        if self.is_in or self.is_out or self.is_stealorfly:
            if self.is_in:
                self.HowEnd = 1
            elif self.is_out:
                self.HowEnd = -2
            elif self.is_stealorfly:
                self.HowEnd = -1
            else:
                print('err')
            return True
        else:
            return False

    def workA(self):
        np.set_printoptions(suppress=True)
        i = 0
        fisrt_time_hold = False
        while not rospy.is_shutdown():
            # print(self.ball_in(), self.ball_out(), self.stealorfly())
            rospy.wait_for_service('/nubot1/BallHandle')
            self.call_Handle(1) # open holding device
            if self.game_is_done() and self.real_resart:
                # print('self.game_is_done()',self.game_is_done())
                self.r = self.cnt_rwd()
                # print('h',self.HowEnd)
                s_ = self.sac_cal.input(self.HowEnd) #out state
                if i > 1:
                    if len(self.s) == 12 and len(s_) == 12:
                        # print('000000000000000000',np.shape(self.s), np.shape(self.a))
                        self.agent.replay_buffer.store_transition(self.s , self.a, self.r, s_, self.done)
                    # print('d',self.done)
                    self.ep_rwd = self.r
                    print('ep rwd value=',self.r)
                    self.end_rate(self.HowEnd)
                i += 1 
                self.s = s_
                self.done = False
                if i > 512: # 64
                    self.agent.learn(i, self.r, self.ep_rwd)
                # self.ready2restart_pub.publish(True)
                # self.ready2restart_pub.publish(False)
                self.real_resart = False
                self.HowEnd = 0
                # print('i want to go in self.restart()')
                self.restart()
                # self.end_rate(self.HowEnd)
                # print('---')
            # elif not self.game_is_done():
                
            else:
                # print('self.game_is_done()',self.game_is_done())
                rospy.wait_for_service('/nubot1/BallHandle')
                if not self.call_Handle(1).BallIsHolding :  # BallIsHolding = 0
                    self.chase(MaxSpd_A)
                    fisrt_time_hold = True
                    rospy.wait_for_service('/nubot1/BallHandle')
                    # do real reset before holding
                    self.ready2restart =False
                    self.is_fly = False
                    self.is_steal = False
                    self.is_stealorfly = False
                    self.is_in = False
                    self.is_out = False
                elif self.call_Handle(1).BallIsHolding : # BallIsHolding = 1
                    global RadHead
                    self.chase(MaxSpd_A)
                    if fisrt_time_hold == True:
                        self.real_resart = True #
                        self.chase(MaxSpd_A)
                        self.show('Touch')
                        self.r = self.cnt_rwd()
                        s_ = self.sac_cal.input(0)                 #state_for_sac
                        if i >= 1:
                            if len(self.s) == 12 and len(s_) == 12:
                                self.agent.replay_buffer.store_transition(self.s, self.a, self.r, s_, self.done)
                            print('step rwd value= ',self.r)
                        self.done = False
                        i += 1  
                        self.s = s_
                        self.a = self.agent.choose_action(self.s)        #action_from_sac
                        rel_turn_ang = self.sac_cal.output(self.a)       #action_from_sac

                        global pwr, MAX_PWR
                        pwr = (self.a[1]+1) * MAX_PWR/2 + 1.3    #normalize
                        # sac]
                                        
                        rel_turn_rad = math.radians(rel_turn_ang)
                        self.RadTurn = rel_turn_rad + self.RadHead
                        fisrt_time_hold = False
                        if i > 512:
                            self.agent.learn(i, self.r, self.ep_rwd)
                    elif fisrt_time_hold == False:
                        self.chase(MaxSpd_A)
                        error = math.fabs (turnHead2Kick(self.RadHead, self.RadTurn))
                        if error > angle_thres : # 還沒轉到    
                            self.turn(self.RadTurn)
                        else : # 轉到
                            self.kick()
                            
    def workB(self):
        # catch = False
        while not rospy.is_shutdown():
            rospy.wait_for_service('/rival1/BallHandle')
            # self.call_Handle(1) #start holding device
            if not self.call_Handle(1).BallIsHolding:  # BallIsHolding = 0
                self.steal_pub.publish(False)
                self.chase_B(MaxSpd_B)
                # catch = False
            else: # BallIsHolding = 1
                # self.chase(MaxSpd_B/4)
                # if not catch:
                    # catch = True
                    # ticks = time.time()
                    # ticks = ticks + 1 # sec # steal time
                # if time.time() > ticks:
                    self.steal_pub.publish(True) # 2C
                # self.show('steal')

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
    s = Strategy('A')
    s.ros_init()
    while not rospy.is_shutdown():
        s.workA()