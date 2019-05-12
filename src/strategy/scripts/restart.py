#!/usr/bin/env python3
import std_srvs.srv import Empty

def restart():
    rospy.wait_for_service('/gazebo/reset_world')
    call_restart = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    call_restart()

    # #對 '...' 之Topic發送資料 10爲緩衝 發送形態爲VelCmd
    # # rospy.init_node('restart', anonymous=True)
    # #初始化ROS並初始化一個節點叫'talker'
    # rate = rospy.Rate(10) # 10hz
    # #update rate of node 
    # # while not rospy.is_shutdown():
    # #無窮迴圈

    # # ref src/nubot_gazebo/scrips/robot_up.sh
    # state = ModelState()
    # state.model_name = 'nubot1'
    # state.pose.position.x = -2
    # state.pose.position.y = 0
    # state.pose.position.z = 0
    # state.pose.orientation.x = 0
    # state.pose.orientation.y = 0
    # state.pose.orientation.z = 0
    # state.pose.orientation.w = 1
    # state.twist.linear.x = 0
    # state.twist.linear.y = 0
    # state.twist.linear.z = 0
    # state.twist.angular.x = 0
    # state.twist.angular.y = 0
    # state.twist.angular.z = 0
    # state.reference_frame = ''
    # restart_pub.publish(state)
    
    # state.model_name = 'rival1'
    # state.pose.position.x = 8.5
    # state.pose.position.y = 0
    # state.pose.position.z = 0
    # state.pose.orientation.x = 0
    # state.pose.orientation.y = 0
    # state.pose.orientation.z = 0
    # state.pose.orientation.w = 1
    # state.twist.linear.x = 0
    # state.twist.linear.y = 0
    # state.twist.linear.z = 0
    # state.twist.angular.x = 0
    # state.twist.angular.y = 0
    # state.twist.angular.z = 0
    # state.reference_frame = ''
    # restart_pub.publish(state)
    
    # state.model_name = 'football,'
    # state.pose.position.x = 8.5
    # state.pose.position.y = 0
    # state.pose.position.z = 0
    # state.pose.orientation.x = 0
    # state.pose.orientation.y = 0
    # state.pose.orientation.z = 0
    # state.pose.orientation.w = 1
    # state.twist.linear.x = 0
    # state.twist.linear.y = 0
    # state.twist.linear.z = 0
    # state.twist.angular.x = 0
    # state.twist.angular.y = 0
    # state.twist.angular.z = 0
    # state.reference_frame = ''
    # restart_pub.publish(state)
    
    # Node: /gazebo
    # URI: rosrpc://james-System-Product-Name:39477
    # Type: std_srvs/Empty
    # Args: 


    #publish()內資料
    # rate.sleep()
    #休息

if __name__ == '__main__':
    restart()