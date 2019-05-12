#!/usr/bin/env python3

#Client
#---How to write "Client"---
#   : if sth happened, then to something
#   : input and then output
# Let it kick 
#   first, check service name  
#       rosservice info /nubot1/Shoot
#       ->  Type: nubot_common/Shoot
#       ->  Args: strength ShootPos
#   Then check service "Type" and input&output
#       rossrv show nubot_common/Shoot
#       ->  float32 strength
#           int64 ShootPos
#           ---
#           int64 ShootIsDone
#           # up side is input, down side is output
#   first line
#       from "src".srv import "Type"
#   '/nubot1/Shoot'， Shoot
#       ='ROS的路徑', "Type"
#---

import sys
import rospy
from nubot_common.srv import *

def ints_client(x, y):
    rospy.wait_for_service('/nubot1/Shoot')
    try:
        call_Shoot = rospy.ServiceProxy('/nubot1/Shoot', Shoot)
        resp1 = call_Shoot(1, 1) #cause 2 input in this Shoot case
        print(resp1.ShootIsDone)
    except rospy.ServiceException:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    ints_client(1,1)