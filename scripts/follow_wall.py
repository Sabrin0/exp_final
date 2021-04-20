#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False

pub_ = None

regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0

state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    # End of wall
    3: 'end of the wall',
}


def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_laser(msg):
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
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
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
    msg = Twist()
    msg.linear.x = 0.25 #0.2
    msg.angular.z = - 0.55 #-0.3
    return msg


def turn_left():
    msg = Twist()
    msg.angular.z = 0.6 #0.3
    return msg


def follow_the_wall():
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
