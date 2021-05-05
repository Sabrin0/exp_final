#!/usr/bin/env python

import roslib; roslib.load_manifest('exp_final')
import rospy
import time
import os
import sys

from exp_final.msg import user
from std_msgs.msg import Bool, String

class console_manager():
    """!@brief This class is the core of the user node. It handles the communication between the command manager 
    and the user interface. 
    """
    def __init__(self):
        """!@brief The construct. It initialize the node and some attributes. It also print a welcome message.
        Attributes
        ----------
        self.color: type `list`
            List of all the possible color related to each specific room
        
        self.state: type `String`
            It represents the current state of the robot. It's initializated as _normal_.

        self.play_pub: `rospy.Publisher()`
            It allows the publication to the topic __userCommand__, custom message. Thought this message the user can send command
            to the robot.
        
        self.play_sub: `rospy.Subscriber()`
            It allows the subscription to the topic '/currentState', type String. Thought the user remains updated
            with the curren state of the robot. 
        """
    
        self.color = ["blue", "red", "green", "yellow", "magenta", "black"]
        self.state = "normal"
        self.play_pub = rospy.Publisher('/userCommand', user, queue_size=1)
        self.play_sub = rospy.Subscriber("/currentState", String, self.callback, queue_size=1)
        rospy.init_node('userPlay')

        print(r"""
                    |\_/|                  
                    | @ @   Woof! 
                    |   <>              _  
                    |  _/\------____ ((| |))
                    |               `--' |   
                ____|_       ___|   |___.' 
               /_/_____/____/_______|

                        Welcome! 
 
                """)

    def callback(self, data):
        """!@brief Callback that receives the messages related to the current state of the robot from the Comand Manager.
        @param data: message over the topic __currentState__ 
        Attributes
        ---
        self.state: type `String`
            It stores the data from the afromentioned topic  
        """
        self.state = data.data
        #rospy.loginfo('HEARD')

    def backUser(self):
        """!@brief Methods that saves the input command _play_ in order to  call the robot to the user.
        This is done by sending the string _play_  over the  topic __currentState__ when the command manager recives the command, 
        it sends the user position as _Nav Stack goal_.
        It also checks if the robot is in the normal state, otherwise it just ignore the input.
        Attributes
        ---
        self.msg_play: ros message
            It contains:
                - play: type `Bool`, it checks if the user inputs the correct command
                - color: type `String`, it stores the color selected by  the user. Initialized as empty string
        
        command: String,
            It stores the `raw_input()` data from the user.
        """
        self.msg_play = user()
        command = raw_input("Please enter 'play' to call the dog: ")
        
        if self.state == 'sleep':
            print('Sorry but the robot is sleeping')
            return self.backUser()

        if command == "play":
            self.msg_play.play = True
            self.msg_play.color = ""
            self.play_pub.publish(self.msg_play)
            rospy.loginfo("Play sent")
            self.state = '' 
            #aprint('pubblico play su topic --> passo allo stato play e cancello goal correnti')
            #return self.GoTo()
        else:
            print('Command not found.')
            self.msg_play.play = False
            #self.msg_play.color = None
            self.play_pub.publish(self.msg_play)
    
            return self.backUser()
    
    def GoTo(self):
        """!@brief Methods that saves the `raw_input()` data representing the _GoTo_ command. \n
        This message, if available, will be sent to the command manager through the topic __userPlay__ .
        Attributes
        ---
         color: type String. 
            Color entered by the user.
        """
        if self.state == 'normal':
            return self.backUser()
            
        print(r"""Please enter a specific color to reach the desired room:

                - blue    -> entrance
                - red     -> closet
                - green   -> living room
                - yellow  -> kitchen
                - magenta -> bathroom
                - black   -> bedroom
                """)

        color = raw_input('Color: ')
        if color in self.color:
            self.msg_play.play = False
            self.msg_play.color = color
            self.play_pub.publish(self.msg_play)
            rospy.loginfo("color sent")
            self.state = ''
        else:
            print('Command Unknown')        
            return self.GoTo()

if __name__ == "__main__":

    # initialize class console_manager()
    play = console_manager()

    while not rospy.is_shutdown():
        #rospy.loginfo('State: %s', play.state)

        if play.state == "normal":
            play.backUser()
        elif play.state == "goto":
            play.GoTo()

        rospy.Rate(20)
    #try: 
    #    play.backUser()
    #    rospy.spin()
    #
    #except rospy.ROSInterruptException:
    #    pass