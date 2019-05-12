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
import math
from strategy_node import *
import random

# global RadKick
# RandomRad = random.uniform(-3.14,3.14)
# print(RandomRad)
# RadKick = RandomRad # from GAFuzzy

def turnHead2Kick(RadHead, RadKick):  
    tmp_kick = RadKick - RadHead

    Rad2Kick = 0.0

    if tmp_kick > math.pi : 
        Rad2Kick = tmp_kick - math.pi*2
    elif tmp_kick <= -math.pi :
        Rad2Kick = tmp_kick + math.pi*2
    else :
        Rad2Kick = tmp_kick

    return Rad2Kick
    
if __name__ == '__main__':
    turnHead2Kick(0, 0)
