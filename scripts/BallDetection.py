#!/usr/bin/env python

"""!
@package dockstring
@section description
OpenCV node
This node is responsible for the ball detection.
In particular, once the ball is detected, it implemnt a simple algorith in order to track the ball untill the goal is reached.
It also publishes on the topic BallState in order to communicate with the cmd_man when the ball is detected and when the robot is next to it.
"""

## Python libs
import sys
import time
import math
import numpy as np
from scipy.ndimage import filters
import imutils
from colorLabeler import ColorLabeler

## OpenCV
import cv2

## Ros libraries
import roslib
import rospy
import actionlib
## ROS messages
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


## Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
#from exp_assignment2.msg import BallState
from exp_final.msg import BallState

VERBOSE = False

def BallDetection(gray, hsv, ballFound):
    
    # detect circle
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, gray.shape[0],
                                param1=50, param2=30,
                                minRadius=0, maxRadius=0)
    
    # initialize color labeler
    #cl = ColorLabeler()

    # exctract radius and center of the circle
    if circles is not None:
        circles = np.uint16(np.around(circles))

    # extract center and radius
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            # mncv.circle(src, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(gray, center, radius, (255, 255, 255), 1)
        color = cl.label(hsv, center, radius)
        print("COLOR DETECTED:", color)
        lowerBound, upperBound = cl.generateBoundaries(color)    
    else:
        lowerBound = None
        upperBound = None
        ballFound = False
        color = None
        print('Ball Not Found')

    cv2.imshow("detected circles", gray)
    cv2.waitKey(2)

    if lowerBound is not None and upperBound is not None:
        ballFound = True
    
    return lowerBound, upperBound, ballFound, color


class image_feature:

    def __init__(self):
        '''! Initialize ros publisher, ros subscriber'''
        rospy.init_node('BallDetection', anonymous=True)
        ## topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel",
                                       Twist, queue_size=1)

        ## flag for ball detected
        self.BallDet_pub = rospy.Publisher("/BallState", BallState, queue_size=1)
        #self.BallDet_pub = rospy.Publisher("BallState", Bool, queue_size=1)
        self.subscriber = rospy.Subscriber("/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)


    def callback(self, ros_data):

        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)
   
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
        #cv2.imshow('test', image_np)
        #cv2.waitKey(2)
        
        ballFound = False
        #greenLower = (50, 50, 20)
        #greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        ### inserire preprocessing ###
        lowerBound, upperBound, ballFound, ballColor = BallDetection(gray, hsv, ballFound)

        #print('##### lower, upper and found: ', lowerBound, upperBound, ballFound)
        # Se ballFound = true -> caccia la palla
        # Se ballColor = false -> palla non ancora trovata quinid procedere 
        if ballFound and not cl.ball[ballColor]:
            #rospy.loginfo('GOAL CANCELLED')
            #client.cancel_all_goals()
            mask = cv2.inRange(hsv, lowerBound, upperBound)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            #cv2.imshow('mask', mask)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None
            ## only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                ## only proceed if the radius meets a minimum size
                if (radius > 10):

                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)

                    vel = Twist()
                    vel.angular.z = -0.002*(center[0]-400)
                    vel.linear.x = -0.01*(radius-100)
                    self.vel_pub.publish(vel)
                    rospy.loginfo("Tracking the ball %d", radius)
                    msg_BallState = BallState()
                    msg_BallState.BallDetected = True
                    msg_BallState.currentRadius = radius
                    msg_BallState.ballColor = ballColor
                    
                    self.BallDet_pub.publish(msg_BallState)

                    if (radius > 95):
                        
                        cl.ball[ballColor] = True
                        print('BallColor:', ballColor, 'reached so flag:', cl.ball[ballColor])
                        rospy.sleep(5)
                
                else:
                    vel = Twist()
                    vel.linear.x = 0.5 #0.5
                    self.vel_pub.publish(vel)
            else: 

                msg_BallState = BallState()
                msg_BallState.BallDetected = False
                rospy.loginfo("Ball lost :(")
                self.BallDet_pub.publish(msg_BallState)

        # update the points queue
        # pts.appendleft(center)
        cv2.imshow('window', image_np)
        cv2.waitKey(2)

        # self.subscriber.unregister()


def main(args):
    '''Initializes and cleanup ros node'''
    
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    cl = ColorLabeler()
    main(sys.argv)