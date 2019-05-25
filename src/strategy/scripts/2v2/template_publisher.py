#!/usr/bin/env python3

# Publisher 
#---How to write "Publisher"---
# 1. When you want to Publish sth. to somewhere ( when u want to get info from somewhere ): 
#   $ rostopic pub /nubot1/nubotcontrol/velcmd nubot_common/VelCmd
#   Check how it pub and work e.g. move 
# Then get its 路徑 and Type（形態）
#   $ rostopic info /nubot1/nubotcontrol/velcmd  
#   -> Type: nubot_common/VelCmd
#---
# In talker fuc below
#   pub = rospy.Publisher('/nubot1/nubotcontrol/velcmd', VelCmd, queue_size=10)
#   pub = rospy.Publisher('ROS的路徑', 形態 , queue_size=10)  
# In while fuc below
#   vec = VelCmd()
#   vec.Vx = 50
#   vec.Vy = 0
#   vec.w = 0
#   pub.publish(vec)
# ---

import rospy
from nubot_common.msg import VelCmd
#from 套件 import VelCmd的形態

def talker():
    pub = rospy.Publisher('/nubot1/nubotcontrol/velcmd', VelCmd, queue_size=10)
    #對 '...' 之Topic發送資料 10爲緩衝 發送形態爲VelCmd
    rospy.init_node('talker', anonymous=True)
    #初始化ROS並初始化一個節點叫'talker'
    rate = rospy.Rate(10) # 10hz
    #update rate of node 
    while not rospy.is_shutdown():
    #無窮迴圈
        vec = VelCmd()
        vec.Vx = 50
        vec.Vy = 0
        vec.w = 0
        pub.publish(vec)
        #publish()內資料
        rate.sleep()
        #休息

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass