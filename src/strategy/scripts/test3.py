#!/usr/bin/env python3

import sys
import rospy
from nubot_common.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('/nubot1/Shoot')
    try:
        call_Shoot = rospy.ServiceProxy('/nubot1/Shoot', Shoot)
        resp1 = call_Shoot(1, 1)
        print(resp1.ShootIsDone)
    except rospy.ServiceException:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    add_two_ints_client(1,1)