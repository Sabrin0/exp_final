#!/usr/bin/env python

import rospy
import actionlib
import random
import time 

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from exp_final.msg import BallState
from nav_msgs.msg import Odometry

def movebase_client():

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    rospy.loginfo('GOAL SENT')
    #wait = client.wait_for_result()
    #rospy.loginfo('Wait done')
    #if not wait:
    #    rospy.logerr("Action server not available!")
    #    rospy.signal_shutdown("Action server not available!")
    #else:s   
    return client.get_result()



class callback:

    def __init__(self):

        self.subOdom = rospy.Subscriber('/odom', Odometry, self.callbackOdom)
        self.subBall = rospy.Subscriber('/BallState', BallState, self.callbackBall)
    
    def callbackOdom(self, dataOdom):
        x = dataOdom.pose.pose.position.x
        y = dataOdom.pose.pose.position.y
        rospy.loginfo('Current position: %d , %d', x, y)

    def callbackBall(self, dataBall):
        ballDetected = dataBall.BallDetected
        ballColor = dataBall.ballColor
        rospy.loginfo('Ball State:', ballDetected)
        if ballDetected:
            result.client.cancel_all_goals()
            rospy.loginfo("Ball found start tracking")
            
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
            



if __name__ == '__main__':

    rospy.init_node('movebase_client_py')
    
        
    while not rospy.is_shutdown():
        #callback = callback()
        #result = movebase_client()
        #rospy.spin()
        movement = targetPosition()
        #x_target, y_target = movement.randomPos()
        movement.GoTo(0, 7)