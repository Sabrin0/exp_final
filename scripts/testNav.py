#!/usr/bin/env python

import rospy
import actionlib
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

class room:
    
    def __init__(self, **entries):
        self.__dict__.update(entries)

room1 = room(name = "entrance", color = "blue", location = None)
room2 = room(name = "closet", color = "red", location = None)
room3 = room(name = "living room", color = "green", location = None)
room4 = room(name = "kitchen", color = "yellow", location = None)
room5 = room(name = "bathroom", color = "magenta", location = None)
room6 = room(name = "bedrom", color = "black", location = None)

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
        rospy.loginfo('Ball State:', ballDetected)
        if ballDetected:
            result.client.cancel_all_goals()
            rospy.loginfo("Ball found start tracking")
            


if __name__ == '__main__':

    rospy.init_node('movebase_client_py')
        
    while not rospy.is_shutdown():
        callback = callback()
        result = movebase_client()
        rospy.spin()