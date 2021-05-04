#!/usr/bin/env python

"""!


@brief This script, implementing a FSM, is the main ROS node.
It's a subcriber of the topic __BallState__, in order to check the current status of the ball and the topic __userCommand__
for the purpose of reaching the Play state whenever it's called.
It's also a publisher, in order to share with the other nodes the current state of the robot.
Regarding the navigation, it's a client of the the _Nav Stack_ action server according to the robot movement. It acts as
a client also for the follow_wall service, so a simple exploring behaviour is implemented.
In this way, by checking several conditions, it's possible to resarch the desired state inside the FSM.   
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
import threading
from collections import OrderedDict
from operator import getitem
## ROS messages
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Float64, Bool
#from exp_final import *
from exp_final.msg import BallState, user
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
# import ros service
from std_srvs.srv import *

## Flag for the ball detection, intialized as `False`
BallDetected = False

## Flag for the ball check, intialized as `False`
BallCheck = False
## Current radius of the ball detected 
currentRadius = 0
## Color of the ball detected, initialized as `None` 
ballColor = None
## Initialization of the follow_wall client
srv_client_wall_follower_ = None
## Play command, initialized as `False`
play = False
## Checking if the the play state is available, initialized as `False`
playAvilable = False
## GoTo command, initialized as `False`
GoTo_room = None


class blueprint:
    """! @brief Class that holds the blueprint of the enviroment. It's used during the navigation since it
        stores the position of the rooms and if they has been visited or not.
    """
    def __init__(self):
        """! 
            @brief The construct. Initialize the dictionary `self.color` and the param `self.findcounter`.

            @param self.color: `dictionary`, in which all the info about the rooms are
            stored
            @param self.findCounter `int`, which count how many time the state Find is called.
        """
        self.color = OrderedDict({
            'blue': {'name':'entrance', 'location': None, 'index':1},
            'red': {'name':'closet', 'location': None, 'index':2},
            'green': {'name':'living room', 'location': None, 'index':3},
            'yellow': {'name':'kitchen', 'location': None, 'index':4},
            'magenta': {'name':'bathroom', 'location': None, 'index':5},
            'black': {'name':'bedroom', 'location': None, 'index':6},
        })
        self.findCounter = 0
        
    def preFind(self):
        """! @brief Method of the class blueprint. It's called by the state Find, in this way, since the robot start from the User position
            it goes first to the last room visited (starting from the first key in the color dictionary) before exporing the enviroment.
            If the findCounter is equal to 0, however it starts from the user position and it is incremented every cicle. \n
            __Please Notice__: given that python2 does not ensure the order of the dictionary a simple method is implemeted to do so.

            @return x,y position of the last know room of the dictionary color  
        """
        
        res = OrderedDict(sorted(self.color.items(),
			key = lambda x: getitem(x[1], 'index')))
		
        locations = []
        known = []
        	
        if self.findCounter == 0:
            print('FIRST IT START FROM HOME')
            self.findCounter +=1
            return [-5, 8]
        
        for key, value in res.items():
            #print('------KEY: ', key)
            #print('******VAL: ', value['location'])
            locations.append(value['location'])
            
        for loc in locations:
            if loc is None:
                break
            known.append(loc)
        
        self.findCounter +=1
        return known[-1]

class pubHandler:
    """! @brief Class that handles publishing on the topic __currentState__.
    """
    def __init__(self):
        """! @brief The construct. Initilize ROS publisher.
        Attributes
        ---
        self.statePub: `rospy.Publisher`,
            It allows the publication over the topic __currentState__, String. With this it's possible to
            publish the current state of the robot.
        """
        self.statePub = rospy.Publisher("currentState", String, queue_size=1)
    
    def pubState(self, state):
        """! Method that provides the pubblication over the topic
        @param state : `String`, the current state of the robot
        Attributes
        ---
        self.statePub: it declares that the node is publishing
        """
        self.statePub.publish(state)
        

class targetPosition:
    """! 
        @brief Class that manages the terget position to set as goal for the _Nav Stack_.
        
    """

    ## home x postion 
    x_home = -5
    ## home y positiom
    y_home = 8
    ## list of random position in the enviroment
    randomPosition = [ 
        [0, 7],
        [-6, 1],
        [3, 2],
        [-6, -3],
        [4, -3],
        [5, -7]
    ]

    def randomPos(self):
        """! Method that provides random position in the enviroment
        @return x_target, y_target: positions to set as goal for the _Nav Stack_
        """
        P = random.randint(0, 5)
        x_target = self.randomPosition[P][0]
        y_target = self.randomPosition[P][1]

        return x_target, y_target
    
    def GoTo(self, x_target, y_target):
        """! 
            @brief Methods that creates an action client move_base with action definition file "MoveBaseAction". 
            After waiting until the action server has started up and started listening for goals it set a new goal
            with the _MoveBaseGoal_ constructor smd sends it. Then it waits for the server to finish performing 
            the action. If the result doesn't arrive, assume the Server is not available.

            @param x_target the x positon set as goal
            @param y_target the y position set as goal

            @return Result of executing the action
        """
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
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

def decision():
    """!@brief Documentation for the function decision()
     
     #return random state between GoTonormal or GoToSleep
    """
    return random.choice(['GoToNormal','GoToSleep'])

def callback_odom(data):
    """!
        @brief Callback for the odom datas provided by the topic __odom__

        @param pos_x: x odometry position of the robot
        @param pos_y: y odometry position of the robot

    """
    global pos_x, pos_y
    pos_x = data.pose.pose.position.x
    pos_y = data.pose.pose.position.y

def callback_check(data):
    """!
        @brief Callback for the odom datas provided by the topic __BallState__
        Once the ball has been detected the robot start tracking it. When the robot is next to the ball,
        it stores its x and y postion in order to update the room location
    
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

    #print('### BallDetected: ', BallDetected, "BallCheck: ", BallCheck, 'CurrentR: ', currentRadius )
    if (BallDetected == True) and (BallCheck == True) and currentRadius > 95:
        
        rospy.loginfo('Tracking the Ball')
        
        #rospy.loginfo('Save ball postion')
        #rospy.sleep(5)
        #room.color[ballColor]['location'] = [pos_x, pos_y]
        #room.lastVisited.append([pos_x, pos_y])
        room.color[ballColor].update( location = [pos_x, pos_y])
        
        print('Name:', room.color[ballColor]['name'], ' Location: ', room.color[ballColor]['location'])

def callback_user(data):
    """!
        @brief Callback for the data provided by the topic __userPlay__
        If it listens the command play, the FSM cancel all goals of the _Nav Stack_ and it goes to the state Play

        @param play: `Bool`, the command play provided by the user 
        @param GoTo_room: `String`, the command _GoTo_ representing the room to reach
    """
    global play, GoTo_room, playAvilable
    play = data.play
    GoTo_room = data.color
    print('play:', play, 'color:', GoTo_room)

    if play and playAvilable:
        playCheck = False
        rospy.loginfo('I heard the user calling me.')
        client.cancel_all_goals()


         
class Normal(smach.State):
    """!@brief Define normal state 
    """

    def __init__(self):
        """!@brief  The construct 
        """
        smach.State.__init__(self, 
                             outcomes=['GoToNormal','GoToSleep','GoToPlay'])
                             #outcomes=['GoToNormal', 'GoToSleep'])
        self.rate = rospy.Rate(1) 
        self.counter = 0
    
    def execute(self,userdata):
        """!@brief Normal state execution
        In the Normal state a random position is sent as goal for the _Nav Stack_.
        If the command `play` is recived the FSM changes state to Play. However after some iterations it
        returns the state Sleep 
        """

        global BallDetected, BallCheck, currentRadius, pos_x, pos_y, play, playAvilable

        rospy.loginfo('Executing state NORMAL')
        #self.counter = random.randint(1,2)
        self.counter = 1
        playAvilable = True
        pub.pubState('normal')
        
        

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
                
            elif self.counter == random.randint(1,3):
                return 'GoToSleep'
            else:
                result = movement.GoTo(x_target,y_target)
                if result:
                    
                    rospy.loginfo('I am arrived')
                    self.counter += 1

        #rospy.loginfo('############ finito loop normal ritorno sleep')
        return 'GoToSleep'

class Sleep(smach.State):
    """!@brief Define Sleep state """
    
    def __init__(self):
        """!@brief The construct    
        """

        smach.State.__init__(self, 
                             outcomes=['GoToNormal','GoToSleep'])
                             
        self.rate = rospy.Rate(200)  # Loop at 50 Hz
        
    def execute(self, userdata):
        """!@brief Sleep state execution. \n 
        
        It sends the home position as goal for the _Nav Stack_.
        Once it's arrived it sleeps and after a while it backs to the state Normal

        @return GoToNormal
        """

        #GoTo = targetPosition()
        rospy.loginfo('--------------------- ')
        rospy.loginfo('Executing state SLEEP ')
        pub.pubState('sleep')
        # Setting the goal home position
        result = movement.GoTo(movement.x_home,movement.y_home)
        if result: 
            rospy.loginfo('i m arrived at home, now i will take a nap')
            time.sleep(3)
            self.rate.sleep()
            return 'GoToNormal'

class Play(smach.State):
    """!@brief Define Play state """

    def __init__(self):
        """!@brief The construct        
        """
        
        smach.State.__init__(self, 
                            outcomes=['GoToNormal', 'GoToFind', 'GoToPlay'])
 
        self.rate = rospy.Rate(200)  # Loop at 50 Hz

    def execute(self, userdata):
        """!
            @brief Play state execution. \n
            First though the _Nav Stack_ the robot comes back to the user. There it waits for the _GoTo_ command and
            if the room is known it reach the desired location. Otherwise it goes to the state Find.
            Moreover the robot waits for the command only for 30 second, if the time expires it goes to the state Normal.
        """
        # Publishin for the user console  
        pub_goto = rospy.Publisher('/stillGoTo', Bool, queue_size=1)
        
        rospy.loginfo("--------------------")
        rospy.loginfo("Executing state PLAY")

        global play, GoTo_room
        self.counter = 0
        # While loop to remain in the state until some conditions are missed
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

            # waiting for user command, after 30 second back to normal:
            start = time.time()
            elapsed = 0
            while (not GoTo_room and elapsed < 30):
                elapsed = start -time.time()
                time.sleep(1)
                print("I am waiting for user command...")
                if elapsed == 10:
                    rospy.loginfo("Too late, back to state normal")
                    
                    return 'GoToNormal'
            
            # Now check if the location is known
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
             
        
class Find(smach.State):
    """!@brief Define Find state """
    
    def __init__(self):
        """!@brief The construct       
        """

        smach.State.__init__(self, 
                             outcomes=['GoToPlay','GoToFind'])
                             
        self.rate = rospy.Rate(200)  # Loop at 50 Hz
        
    def execute(self, userdata):
        """! 
            @brief Find state execution. \n
            If it's the first iteration the follow_wall service is called from the user position.
            Otherwise the robot first goes to the last known postion and the it starts exploring. \n
            When it reaches the desired location the robots save the position and it returns the state Play.
            After a while, if the robot does not find the desired room, it comes however to the state Play. 
        """
        global GoTo_room, ballColor, srv_client_wall_follower_
        srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
        rospy.loginfo('--------------------- ')
        rospy.loginfo('Executing state FIND ')
        # first go to the last known position
        go = room.preFind()
        result = movement.GoTo(go[0],go[1])
        if result:
        # Setting the goal home position
            start = time.time()
            elapsed = 0
            while room.color[GoTo_room]['location'] is None:
                elapsed = time.time() - start
                if elapsed > 300:
                    GoTo_room = ''
                    resp = srv_client_wall_follower_(False)
                    return 'GoToPlay'
                sent = False
                if not sent:
                    sent = True
                    print('reaching the room: ', room.color[GoTo_room]['name'] )
                resp = srv_client_wall_follower_(True)
            GoTo_room = ''
            resp = srv_client_wall_follower_(False)
            rospy.sleep(1)
            return 'GoToPlay'

            

def main():
    """!
        @brief Main entry of the node. Initialize the ros node, the severals subscriptions and pubblications.
    """
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
    ## Initialization of the class blueprint
    room = blueprint()
    ## Initialization of the class targetPosition
    movement = targetPosition()
    ## Initialization of the class pubHandler
    pub = pubHandler()
    main()