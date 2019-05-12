#!/usr/bin/env python3
# license removed for brevity
import rospy
from nubot_common.msg import VelCmd

def talker():
    pub = rospy.Publisher('/nubot1/nubotcontrol/velcmd', VelCmd, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        vec = VelCmd()
        vec.Vx = 50
        vec.Vy = 0
        vec.w = 0
        pub.publish(vec)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass