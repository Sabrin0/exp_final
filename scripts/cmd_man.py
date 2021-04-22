#!/usr/bin/env python

"""!
@author Luca Covizzi 
@file cmd_man.py
@mainpage Exprob Assignment 2
@section Description

This script, implementing a FSM, is the main ROS node.
It's a subcriber of the topic BallState, in order to check the current status of the ball.
It's also a publisher, due to the function head_control in order to move the robot head.
And moreover it's a client of the the action server according to the robot movement.
In this way, by checking several condition, it's possible to resarch the desired state inside the FSM.   
"""

from __future__ import print_function

## Imports
import roslib; roslib.load_manifest('exp_final')
import rospy
import smach
import smach_ros
import time
import math
import random
import sys
import rospy
import actionlib
from collections import OrderedDict
## ROS messages
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Float64, Bool
#from exp_final import *
from exp_final.msg import BallState, user
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
# import ros service
from std_srvs.srv import *

## Define home x position
# @param x_home The fixed home coordinate x 
#x_home = 5

## Define home y position 
# @param x_home The fixed home coordinate x
#y_home = 8

## Initial state of the ball status
## @param BallDetected, bool flag for the ball detection
BallDetected = False

## @parama BallCheck, bool flag for the ball check  
BallCheck = False
currentRadius = 0
## Ball Color
ballColor = None

srv_client_wall_follower_ = None

play = False
playAvilable = False
GoTo_room = None

## Room class:
class blueprint:
    def __init__(self):

        self.color = OrderedDict({
            'blue': {'name':'entrance', 'location': None},
            'red': {'name':'closet', 'location': None},
            'green': {'name':'living room', 'location': None},
            'yellow': {'name':'kitchen', 'location': None},
            'magenta': {'name':'bathroom', 'location': None},
            'black': {'name':'bedroom', 'location': None},
        })
        self.findCounter =0

    def preFind(self):
        locations = []
        known = []

        if self.findCounter == 0:
            self.findCounter += 1
            print('######## FIRST ITERATION PARTO DA CASA')
            return [-5, 8]
        
        for key, value in self.color.items():
            print('------KEY: ', key)
            print('******VAL: ', value['location'])
            #if value['location'] is not None:
            #    explored.append(value['location'])
            locations.append(value['location'])
        print('LOCATION: ', locations)
        for loc in locations:
            if loc is None:
                break
            known.append(loc)

        self.findCounter += 1
        print('!!!!!!!!!!!!!!!!! known', known)
        print('----------------l last known: ', known[-1])
        return locations[-1]

class pubHandler:

    def __init__(self):
        self.statePub = rospy.Publisher("currentState", String, queue_size=1)
    
    def pubState(self, state):
        self.statePub.publish(state)
        

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
        if x_target == self.x_home:
            rospy.loginfo('Back home')
        else:
            rospy.loginfo('i m going to x: %d y: %d',goal.target_pose.pose.position.x,goal.target_pose.pose.position.y)
        client.send_goal(goal)
        print('I am moving...')
        wait = client.wait_for_result()           

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()


## Action client for the action server dedicated to the movment
#client = actionlib.SimpleActionClient('/robot/reaching_goal_robot', exp_assignment2.msg.PlanningAction)
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
# Functions 
def decision():
    """!@brief Documentation for the function decision()
     
     It returns random state between GoTonormal or GoToSleep
    """
    return random.choice(['GoToNormal','GoToSleep'])

def callback_odom(data):
    global pos_x, pos_y
    pos_x = data.pose.pose.position.x
    pos_y = data.pose.pose.position.y

def callback_check(data):
    """!@brief Documentation for the function callback_check()
    
    This function subscribes to the topic BallState.
    If the message related to the ball detection is setted to true, the action server for the robot movement is stopped and the state PLAY starts.
    While the ball is detected, the values of the radious is updated and if it exceeds a treshold the head starts to rotate.
    
    @param BallDetected Bool
    @param BallCheck Bool
    @param currentradious Float64
    """
    global BallDetected, BallCheck, currentRadius, ballColor, pos_x, pos_y
    BallDetected = data.BallDetected
    currentRadius = data.currentRadius
    ballColor = data.ballColor

    if (BallDetected == True) and (BallCheck == False):
        BallCheck = True
        #rospy.loginfo("Ball Detected! Start tracking ")
        #client.cancel_all_goals()

    print('### BallDetected: ', BallDetected, "BallCheck: ", BallCheck, 'CurrentR: ', currentRadius )
    if (BallDetected == True) and (BallCheck == True) and currentRadius > 90:
        
        #rospy.loginfo('Tracking the Ball')
        
        rospy.loginfo('Save ball postion')
        #rospy.sleep(5)
        #room.color[ballColor]['location'] = [pos_x, pos_y]
        room.color[ballColor].update( location = [pos_x, pos_y])
        
        print('Name:', room.color[ballColor]['name'], ' Location: ', room.color[ballColor]['location'])

def callback_user(data):
    global play, GoTo_room, playAvilable
    play = data.play
    GoTo_room = data.color
    print('play:', play, 'color:', GoTo_room)

    if play and playAvilable:
        playCheck = False
        rospy.loginfo('I heard the user calling me.')
        client.cancel_all_goals()


         
class Normal(smach.State):
    """!@brief Define normal state """

    def __init__(self):
        """!@brief Initialization of the functioin        
        """
        smach.State.__init__(self, 
                             outcomes=['GoToNormal','GoToSleep','GoToPlay'])
                             #outcomes=['GoToNormal', 'GoToSleep'])
        self.rate = rospy.Rate(1) 
        self.counter = 0
    
    def execute(self,userdata):
        """!@brief Normal state execution
        
        In the NORMAL state a random position is sent to the Action Server go_to_point_action.
        If the ball is detected the goal is cancelled
        """

        global BallDetected, BallCheck, currentRadius, pos_x, pos_y, play, playAvilable

        rospy.loginfo('Executing state NORMAL')
        #self.counter = random.randint(1,2)
        self.counter = 1
        playAvilable = True
        pub.pubState('normal')
        #goal = exp_assignment2.msg.PlanningGoal()
        #goal = MoveBaseGoal()
        #GoTo = targetPosition()
        

        while not rospy.is_shutdown():  
            
            #x_target, y_target = GoTo.randomPos()
            ## If the Ball is Detcted, go to PLAY
            # @return GoToPlay
            x_target, y_target = movement.randomPos()
            ## After some NORMAL state iteration, go to SLEEP mode
            # @return GoToSleep
            if play:
                rospy.loginfo('RITORNO <PLAY>')
                return 'GoToPlay'
                
            elif self.counter == 2:
                return 'GoToSleep'
            else:
                result = movement.GoTo(0,7)
                if result:
                    
                    rospy.loginfo('I am arrived')
                    self.counter += 1

        rospy.loginfo('############ finito loop normal ritorno sleep')
        return 'GoToSleep'

class Sleep(smach.State):
    """!@brief Define Sleep state """
    
    def __init__(self):
        """!@brief Initialization of the functioin        
        """

        smach.State.__init__(self, 
                             outcomes=['GoToNormal','GoToSleep'])
                             
        self.rate = rospy.Rate(200)  # Loop at 50 Hz
        
    def execute(self, userdata):
        """!@brief Sleep state execution 
        
        It sends a position to the server Go_to_Point_action in order to back home.
        After a while it backs to the state NORMAL

        @return GoToNormal
        """

        #GoTo = targetPosition()
        rospy.loginfo('--------------------- ')
        rospy.loginfo('Executing state SLEEP ')
        ## Setting the goal home position
        result = movement.GoTo(movement.x_home,movement.y_home)
        if result: 
            rospy.loginfo('i m arrived at home, now i will take a nap')
            time.sleep(3)
            self.rate.sleep()
            return 'GoToNormal'

class Play(smach.State):
    """!@brief Define Play state """

    def __init__(self):
        """!@brief Initialization of the functioin        
        """
        
        smach.State.__init__(self, 
                            outcomes=['GoToNormal', 'GoToFind', 'GoToPlay'])
 
        self.rate = rospy.Rate(200)  # Loop at 50 Hz

    def execute(self, userdata):

        ## Publishin for the user console  
        pub_goto = rospy.Publisher('/stillGoTo', Bool, queue_size=1)
        """!@brief Play state execution

        It moves the robots to the ball while it's detected.
        Once arrived next to the ball, the robort starts to check around.
        """
        rospy.loginfo("--------------------")
        rospy.loginfo("Executing state PLAY")

        global play, GoTo_room
        self.counter = 0
        ## While loop to remain in the state until some conditions are missed
        while not rospy.is_shutdown():
            
            
            #first go back to the user
            result = movement.GoTo(movement.x_home,movement.y_home)
            if result: 
                pub.pubState('goto')
                rospy.loginfo('i m arrived to the user')
                time.sleep(1)
                self.rate.sleep()
                #self.counter += 1

            ## After some iteration go to state normal
            if self.counter == random.randint(2,4):
                
                return 'GoToNormal'

            ## waiting for user command, after 30 second back to normal:
            start = time.time()
            elapsed = 0
            while (not GoTo_room and elapsed < 30):
                elapsed = start -time.time()
                time.sleep(1)
                print("I am waiting for user command...")
                if elapsed == 30:
                    rospy.loginfo("Too late, back to state normal")
                    
                    return 'GoToNormal'
            
            ## Now check if the location is known
            if room.color[GoTo_room]['location'] is not None:
                result = movement.GoTo(room.color[GoTo_room]['location'][0],room.color[GoTo_room]['location'][1])
                if result:
                    
                    rospy.loginfo('I m arrived at %s:', room.color[GoTo_room]['name'])
                    # per evitare loop  
                    GoTo_room = "" 
                    time.sleep(1)
                    self.counter += 1
        
            else:
                
                rospy.loginfo('######## ROOM UNKNOWN')
                #pub.pubState('')
                return 'GoToFind'
        
        return 'GoToNormal'
            ## Start moving the head if the robot is near to the ball
            # @param currentRadious float passsed
            #if (currentRadius > 90):
                # rospy.loginfo('muovo la testa')
                # head_control()
                # Save ball coordiantes
            
            ## Back to state normal if the ball is missed
            # @param BAllDetected bool 
            #if (BallDetected == False):
            #    BallCheck = False
           #     rospy.loginfo('WOOF! Ball missed :(')
           #     return 'GoToNormal'

            #time.sleep(3) 
        
class Find(smach.State):
    """!@brief Define Sleep state """
    
    def __init__(self):
        """!@brief Initialization of the functioin        
        """

        smach.State.__init__(self, 
                             outcomes=['GoToPlay','GoToFind'])
                             
        self.rate = rospy.Rate(200)  # Loop at 50 Hz
        
    def execute(self, userdata):
        global GoTo_room, ballColor, srv_client_wall_follower_
        srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
        rospy.loginfo('--------------------- ')
        rospy.loginfo('Executing state FIND ')
        ## first go to the last known position
        go = room.preFind()
        result = movement.GoTo(go[0],go[1])
        if result:
        ## Setting the goal home position
            while room.color[GoTo_room]['location'] is None:
                print('reaching the room: ', room.color[GoTo_room]['name'] )
                resp = srv_client_wall_follower_(True)
            GoTo_room = ''
            resp = srv_client_wall_follower_(False)
            rospy.sleep(1)
            return 'GoToPlay'

            

def main():

    ## Initialization of the node
    rospy.init_node('smach_state_machine')
    ## Subscribing to Ball State topic
    rospy.Subscriber('/BallState', BallState, callback_check)
    client.wait_for_server()
    
    #Subscribe to odometry topic
    rospy.Subscriber("odom", Odometry, callback_odom)
    #subscribe to user topic 
    rospy.Subscriber('/userCommand', user, callback_user)

    ## Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    ## Open the container
    with sm:
        ## Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'GoToSleep':'SLEEP',
                                            'GoToPlay':'PLAY',
                                            'GoToNormal':'NORMAL'})

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'GoToSleep':'SLEEP', 
                                            'GoToNormal':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(),
                                transitions={'GoToNormal':'NORMAL',
                                             'GoToFind':'FIND',
                                             'GoToPlay':'PLAY'})
        
        smach.StateMachine.add('FIND', Find(),
                                transitions={
                                    'GoToPlay':'PLAY',
                                    'GoToFind':'FIND'})


    ## Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    ## Execute the state machine
    outcome = sm.execute()

    ## Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    room = blueprint()
    movement = targetPosition()
    pub = pubHandler()
    main()