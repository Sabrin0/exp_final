#! /usr/bin/env python

import rospy
import time
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *
from geometry_msgs.msg import Twist

import math

pub = None
srv_client_wall_follower_ = None

class targetPosition:

    x_home = -5
    y_home = 8
    
    randomPosition = [ 
        [0, 7],
        [-6, 1],
        [3, 2],
        [-6, -3],
        [4, -3],
        [5, -7]
    ]

    def randomPos(self):
        P = random.randint(0, 5)
        x_target = self.randomPosition[P][0]
        y_target = self.randomPosition[P][1]

        return x_target, y_target
    
    def GoTo(self, x_target, y_target):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_target
        goal.target_pose.pose.position.y = y_target
        goal.target_pose.pose.orientation.w = 2.0
        #goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo('i m going to x: %d y: %d',goal.target_pose.pose.position.x,goal.target_pose.pose.position.y)
        client.send_goal(goal)
        print('Waiting...')
        wait = client.wait_for_result()           

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()

def main():
    global srv_client_wall_follower_
    
    rospy.init_node('test')
    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        #resp = srv_client_wall_follower_(False)
        result = movement.GoTo(0,7)
        if result:
            print('Arrivato atttivo wall per 30 secondi')
            start = time.time()
            elapsed = 0
            
            while elapsed < 30:
                resp = srv_client_wall_follower_(True)
                elapsed = time.time() - start 
                time.sleep(1)
                print('----> time: ', int(elapsed))

            
            print('FALSE')
            resp = srv_client_wall_follower_(False)
                    

        rate.sleep()

if __name__ == '__main__':

    movement = targetPosition()
    main()