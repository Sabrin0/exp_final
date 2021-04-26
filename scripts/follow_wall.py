#! /usr/bin/env python

"""!
    @section Description
    This is a ROS server that provide navigation though the walls once it has been activated by the command manager. \n
    Flag for server activation.
   Simple dictionary that stores the laser field of view into five separate regions
"""


import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math

active_ = False

pub_ = None

## Initialization of the dictionay regions_ 
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}

## Initialization of the global variable state_.

state_ = 0

## Initialization of the dictionary state_dict_.
# It represents the three possible states of the behaviour.

state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    # End of wall
    3: 'end of the wall',
}


def wall_follower_switch(req):
    """!
        @brief Function that handle the server call.
        @return res:
             It returns a `Bool` value in order to activate or not the server
        
    """
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_laser(msg):
    """!
        @brief Callback that handle the data recived from the laser .
        
        @param regions_: Dictionary that stores the ranges data [m] from the `LaserScan` into five different regions.
            Please notice: It's stores only the minimum value in order to give tha maximum priority 
            to the nearest point of the object detected.
    """
    
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action()


def change_state(state):
    """!
        @brief Function thar provides the switch among the three different states.

        @param[in] state_ 
        @param state_dict_ 
        
    """
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    """! @brief Function that returns the state to perform depending on which regions of the laser is involved.
        @param state_description: `String`, simple description of the current state 
        @param d0: `flaot`, minor treshold
        @param d: `float`, major treshold
    """
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    d0 = 0.6 #0.75 // 0.5
    d = 0.9 #1.25 // 0.75 

    if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d0: #and regions['right'] < d0:
        state_description = 'case 8 - fleft and fright'
        change_state(2)
    elif regions['front'] > d and regions['fright'] < d and regions['right'] < d:
        change_state(1)
    #elif regions['front'] > d0 and regions['fright'] < d and regions['right'] < d0:
    #    state_description = 'case 9 end of wall'
    #    change_state(3)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
    #print(state_description)


def find_wall():
    """! @brief Function that publishes velocities on the topic __cmd_vel__, It allows the robot to reach the wall.
        @param msg.linear.x: `Twist()`, field of the ROS msg representing the linear velocity along x
        @param msg.angular.z: `Twist()`, field of the ROS msg representing the angular velocity along z
        @return msg    
    """
    msg = Twist()
    msg.linear.x = 0.25 #0.2
    msg.angular.z = - 0.55 #-0.3
    return msg


def turn_left():
    """! @brief Function that publishes velocities on the topic __cmd_vel__, It allows the robot to turn left and avoid obstacles.
        @param msg.angular.z: `Twist()`, field of the ROS msg representing the angular velocity along z 
        @return msg    
    """
    msg = Twist()
    msg.angular.z = 0.6 #0.3
    return msg


def follow_the_wall():
    """! @brief Function that publishes velocities on the topic __cmd_vel__, It allows the robot to follow the wall.
        @param msg.linear.x: `Twist()`, field of the ROS msg representing the linear velocity along x
        @param msg.angular.z: `Twist()`, field of the ROS msg representing the angular velocity along z
        @return msg    
    """
    global regions_
 
    msg = Twist()
    msg.angular.z = 0.2
    msg.linear.x = 0.3  
    return msg

# End of wall
def end_wall():
    msg = Twist()
    msg.angular.z = -0.4
    return msg

def main():
    """! @brief Main program entry of the node, which is initilized as ROS service, publisher on the topic __cmd_vel__ and subriber on topic
    __scan__. \n
    Inside a loop, It checks the current state provided in order to switch between the three differen states.  
    """
    global pub_, active_

    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
    #print('-------> active: ', active_)
        if not active_:
            rate.sleep()
            print('server down')
            continue
        else:
            msg = Twist()

            if state_ == 0:
                msg = find_wall()

            elif state_ == 1:
                msg = turn_left()

            elif state_ == 2:
                msg = follow_the_wall()
        # End of Wall
        #elif state_ == 3:
        #    msg = end_wall()
            else:
                rospy.logerr('Unknown state!')
            
        #print('--------> msg:' , msg)
            pub_.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()
