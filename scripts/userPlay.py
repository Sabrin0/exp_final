#!/usr/bin/env python

import roslib; roslib.load_manifest('exp_final')
import rospy
import time
import os
import sys

from exp_final.msg import user

class console_manager():
    def __init__(self):
        # check later if the color passed is ok
        self.color = ["blue", "red", "green", "yellow", "magenta", "black"]
       
        # initialize node as publisher
        self.play_pub = rospy.Publisher('/userCommand', user, queue_size=1)
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

    def backUser(self):
        self.msg_play = user()
        command = raw_input("Please enter 'play' to call the dog: ")

        if command == "play":
            self.msg_play.play = True
            self.play_pub.publish(self.msg_play)
            rospy.loginfo("Play sent")
            #aprint('pubblico play su topic --> passo allo stato play e cancello goal correnti')
            return self.GoTo()
        else:
            print('Command not found.')
            self.msg_play.play = False
            self.play_pub.publish(self.msg_play)
    
            return self.backUser()
    
    def GoTo(self):
        
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
            #self.msg_play.play = False
            self.msg_play.color = color
            self.play_pub.publish(self.msg_play)
            rospy.loginfo("color sent")
        else:        
            return self.GoTo()

if __name__ == "__main__":

    play = console_manager()
    while not rospy.is_shutdown():
        play.backUser()
    #try: 
    #    play.backUser()
    #    rospy.spin()
    #
    #except rospy.ROSInterruptException:
    #    pass