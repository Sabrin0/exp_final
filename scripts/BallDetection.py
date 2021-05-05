#!/usr/bin/env python

"""!

This node is responsable for the ball detection. 
There are two main phases:

- __pre-processing__: Due to the module _colorLabeler_ the robot is able to detect any ball in the enviroment and recognize its color.
    Once the ball is detected, the afromentioned module also returns the features for the specific color in order to procede with the tracking
- __tracking__: It implemnts a simple algorith in order to track the ball untill the ball is reached, then publishes on the topic __cmd_vel__ 
    in order to reach the ball position.

It also publishes on the topic __BallState__ in order to communicate with the cmd_man whenever the ball is detected and the robot is next to it.
By subscribing on the topic __currentState__, the robot avoids the tracking when the robot is in the sleep state. Also it remembers which ball has been already detected so, even in this case, it avoids the tracking.
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
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
#from exp_assignment2.msg import BallState
from exp_final.msg import BallState

VERBOSE = False

class collision():

    def __init__(self):
        self.stop = False
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        self.d = 0.4

    def clbk_laser(self, msg):
        print('--------> MIN:', min(min(msg.ranges), 10))
        if min(min(msg.ranges), 10) < self.d:
            self.stop = True
        else:
            self.stop = False 

def BallDetection(gray, hsv, ballFound):
    """!
        Function that recognize any ball in the enviroment by using th _OpenCV_ function `cv2.HoughCircles(image, method, dp, minDist)`.
        Once the ball has been detected it calls the module colorLabelr and its method generateBoundaries in order to return the required features.
        
        @param gray: image in grayscale
        @param hsv: image in hsv colorspace
        @param ballFound: `Bool`, It represents if the ball has been found or not

        @return lowerBounds: `np.array`, lower mask of the color detected
        @return upperBound: `np.array`, upper mask of the color detected 
        @return ballFound: `Bool`, it indicate wheter the ball is dected or not
        @return color: `String`, it represents the color of the ball detected
            
    """
    
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

    #cv2.imshow("detected circles", gray)
    #cv2.waitKey(2)

    if lowerBound is not None and upperBound is not None:
        ballFound = True
    
    return lowerBound, upperBound, ballFound, color


class image_feature:

    def __init__(self):
        '''! The construct. Initialize the node, ros publisher, ros subscriber.
            Attributes
            ---
            self.image_pub: `rospy.Publisher()`,
                It allows the publication to the topic __output/image_raw/compressed__, CompressedImage.

            self.vel_pub: `rospy.Publisher()`, 
                It allows the publication to the topic __cmd_vel__, Twist. With this it's possible to move the robot toward the ball  
            
            self.BallDet_pub: `rospy.Publisher()`,
                It allows the publication to the topic __BallState__, custom message. With this it's possible to send to the cmd_man specif istructions.

            self.subscriber: `rospy.Subscriber()`,
                It allows the subscription to the topic __camera1/image_raw/compressed__, CompressedImage.
                With this it's possible to receive and then process the image from the camera.

            self.state_sub: `rospy.Subscriber()`,
                It allows the subscription to the topic __currentState__, String. With this it's possible to track or not the ball
                depending on the current robot state
        '''
        rospy.init_node('BallDetection', anonymous=True)
        ## Publisher topic /output/image_raw/compressed
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        ## Publisher topic /cmd_vel
        self.vel_pub = rospy.Publisher("/cmd_vel",
                                       Twist, queue_size=1)
        ## Publisher topic /BallState
        self.BallDet_pub = rospy.Publisher("/BallState", BallState, queue_size=1)
        ## Subscriber topic /camera1/image_raw/compressed
        self.subscriber = rospy.Subscriber("/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)
        ## Subscriber topic /currentState
        self.state_sub = rospy.Subscriber("/currentState", String, self.callbackState, queue_size=1)

    def callbackState(self, data):
        """!@brief Callback that receives the messages related to the current state of the robot from the cmd_man.
        @param data: message over the topic __currentState__ 
        Attributes
        ---
        self.state: type `String`
            It stores the data from the afromentioned topic  
        """
        self.state = data.data
        rospy.loginfo('Current State: %s' %self.state)

    def callback(self, ros_data):

        """!Callback function of subscribed topic. 
            Here images get converted and features detected. Once the ball is dectetec it provides a simple alghorithm in order to follow
            the ball, if neither the ball has been already detecter and nor the robot is in the state sleep.
            Once the robot is next to the ball, the corresponding key of the dictionary `cl.ball` is set to `True`. It also publish the _currentRadius_
            and the current _color_ of ball on the topic __BallStatus__.
        """
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
        rospy.loginfo('---------- Stop: %s' %laser.stop) 
        if ballFound and not cl.ball[ballColor] and (self.state != 'sleep') and not laser.stop: 
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
                if (radius > 5): #10

                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)

                    vel = Twist()
                    vel.angular.z = -0.008*(center[0]-400) #0.002
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
                        #rospy.sleep(5)
                
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
    """! Main program entry, it initializes and cleans up ros node.
    """
    
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    #client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    ## Initiliazation class ColorLabeler
    laser = collision()
    cl = ColorLabeler()
    main(sys.argv)