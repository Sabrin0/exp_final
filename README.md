![Unige](/assets/Unige.png)
# Experimental Final Assignment
### Table of Contents 
- [Experimental Final Assignment](#experimental-final-assignment)
    - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Software Architecture](#software-architecture)
    - [Description](#description)
    - [Nodes](#nodes)
      - [Command Manager (cmd_man.py)](#command-manager-cmd_manpy)
        - [State Normal](#state-normal)
        - [State Sleep](#state-sleep)
        - [State Play](#state-play)
        - [State Find](#state-find)
      - [OpenCV (BallDetection.py)](#opencv-balldetectionpy)
      - [Follow Wall Service (follow_wall.py)](#follow-wall-service-follow_wallpy)
      - [User Interface (userPlay.py)](#user-interface-userplaypy)
  - [Package and File List](#package-and-file-list)
  - [Installation and Running Procedure](#installation-and-running-procedure)
  - [System Features](#system-features)
  - [System Limitation and Possible Solutions](#system-limitation-and-possible-solutions)
  - [Author & Contact](#author--contact)


## Introduction
![enviroment](/assets/enviroment.png "Enviromen top view")
*Fig. 1: Enviroment Top View*

This project is the further work of the previous [assignment](https://github.com/Sabrin0/Assignment2-Experimental-RoboticS-LAB.git) of *Experimental robotics lab*.
Due to the [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) and the [ROS Navighation Stack](http://wiki.ros.org/navigation) the  wheeled robot is able to move autonomously.  The main purpose of this project is to implement a FSM which allows the robot to learn the environment and interact with the user in order to reach the desired room. Each colored ball represents a different room (as shown below) and at the beginning of the simulation all the locations are unknown. It’s up to the robot to navigate and store the informations about the environment though a simple exploring algorithm.


## Software Architecture

### Description

 ![rosgraph](/assets/rosgraph.png)
*Fig.2: Rosgraph*

As mentioned before, the current software architecture takes up the previous one. The main node is still the Command Manger, which implements a FSM.
Basically the wheeled robot, equipped with an Hokuyo range finders and a RGB camera, follows four different behaviors:

- **Normal**: in which the robot moves randomly in the environment 
- **Sleep**: in which the robot comes back to the user position
- **Play**: in which the robot interacts with the user in order to receive command and reach specific rooms
- **Find**: in which the robot search the unkown rooms and stores their location

The navigation, including the global and the local path planning and the obstacles avoidance is provided by the package [ROS Navighation Stack](http://wiki.ros.org/navigation) while the [gmapping](http://wiki.ros.org/gmapping) supplies a 2-D occupancy grid map from laser and pose data collected by a mobile robot. In any case, the goal is set by the Command Manager depending on the current situation.

### Nodes

#### Command Manager ([cmd_man.py](https://github.com/Sabrin0/exp_final/blob/main/scripts/cmd_man.py))
![FSM diagram](/assets/FSM.png)
*Fig.3: FSM Diagram*

##### State Normal
In this state the robot moves randomly in the environment by sending several positions as goal to the Navigation Server. Here, whenever a ball is detected the robot start tracking it without cancelling the current goal. This is possible due to the fact that the open cv node controls directly the actuators by publishing on the topic *cmd_vel*. Once the robot is close enough to the ball, it saves the current position of the ball by subscribing to the topic *odom*. After some iteration, the robot goes in the state **Normal**.
The command manager also subscribes to the topic *userCommand*, so whenever it receives the command `play` it cancels the current goal and goes into the state **Play**.

##### State Sleep
In this state the robot simply comes back to the user position located at the coordinates x = -8 and y = 8. Once arrived, after a while it returns the state **Normal**.

##### State Play

Once the command manager receives the input command `play` from the **User Interface**, the robot comes back to the home position and waits for a *GoTo* command. If the location is known the robot will reach it, otherwise it goes to the state **Find**. Moreover, the robots waits for the instructions 30 seconds then if no command is specified the robot goes back to the state **Normal**.

##### State Find
This state is called from the **Play** one, only if the location proposed by the user is unknown. It acts as a _client_ for the [follow_wall](https://github.com/Sabrin0/exp_final/blob/main/scripts/follow_wall.py) service by which the robot starts to explore the environment, following the walls. 
The main purpose of this exploration is to find the desired room and store the information about its coordinates. After 5 minutes, if the required room has not be found, the robot goes back to the state **Play**. In any case, the other unknown rooms are tracked (if found).
Given the fact that the robot always starts exploring from the user position and whereas the robot follows the wall in a *anti-clockwise direction* some room would ever find (for instance the bedroom).
For this reason, a simple *pre-find* algorithm has been implemented in order to go first at the last known room visited (*always from an anti-clockwise direction pov*). In this phase, the algorithm iterates among the room indexes (*from blue to black*) and set as navigation goal the location that precedes the first unknown one found.

#### OpenCV ([BallDetection.py](https://github.com/Sabrin0/exp_final/blob/main/scripts/BallDetection.py))

This node manages the ball detection and therefore the tracking by processing the image received from the robot RGB camera. It's characterized by two main phases: 

-  **[Pre-processing](https://github.com/Sabrin0/exp_final/blob/main/scripts/colorLabeler.py)** : in which an algorithm performs object and color detection, in order to identify any ball in the environment and recognizes its color. Then it returns the upper and lower masks required in the next phase.

- **Tracking**: Depending on several conditions, for instances the current state  of the robot (updated by the topic *currentState*) and if one specific ball has not been already detected, the algorithm in this phase starts tracking the ball.

This node subscribes to two main topics:

- *BallState*: By which the command manager is up to date regarding the position of the robot wrt the tracked ball. Once the robot is sufficiently near to the ball, in the command manager the specific location is stored.

- *cmd_vel*: By which the robot movement is controlled in order to reach the ball. Publishing directly on the aforementioned topic, the Navigation Stack *loses its priority*. 

Moreover by subscribing to the topic */scan*, if an obstacle is detected close to the robot the tracking is interrupted. This prevents getting stuck against the wall.

#### Follow Wall Service ([follow_wall.py](https://github.com/Sabrin0/exp_final/blob/main/scripts/follow_wall.py))
![laser](/assets/laser.jpeg)
*Fig.5: Laser fov subdivided into 5 regions*

This node is a ROS server that provides the exploration of the environment and it is activated by the command manager, which acts as a Client, inside the state **Find**.
It relies on the laser data provided by the topic *Scan*. In particular the *fov* of the sensor is divided into 5 regions, then depending on which of them detect an obstacle (and at what distance) the exploration switches among three different navigation state:

- **find wall**: the robot looks for a near wall with a circular motion
- **follow wall**: the robot follows the wall detected keeping a certain distance
- **turn left**: the robot simply turns left (only angular velocity among z) in order to avoid getting stuck.

The navigation is ensured by publishing on the topic *cmd_vel*.

#### User Interface ([userPlay.py](https://github.com/Sabrin0/exp_final/blob/main/scripts/userPlay.py))
```
user@hostname$          |\_/|                  
                        | @ @   Woof! 
                        |   <>              _  
                        |  _/\------____ ((| |))
                        |               `--' |   
                    ____|_       ___|   |___.' 
                /_/_____/____/_______|
                            Welcome! 

Please enter 'play' to call the dog:
```
*Output of the UI*

This node provides the communication between the user and the command manager though the shell by publishing on the topic *userState* and moreover it is available only when the robot is in the state **Normal**.
First of all, the user can call back the robot by entering the string `play`. Once the robot has reached the user position, it’s possible to enter a new command, in details the color related to a specific room, in order to make the let the robot move to the desired location.

## Package and File List
```
├── CMakeLists.txt
├── Doxyfile
├── README.md
├── assets
├── config
│   └── sim2.rviz
├── docs
├── launch
│   ├── gmapping.launch
│   ├── move_base.launch
│   └── simulation.launch
├── msg
│   ├── BallState.msg
│   └── user.msg
├── package.xml
├── param
│   ├── base_local_planner_params.yaml
│   ├── costmap_common_params.yaml
│   ├── global_costmap_params.yaml
│   ├── local_costmap_params.yaml
│   └── move_base_params.yaml
├── scripts
│   ├── BallDetection.py
│   ├── cmd_man.py
│   ├── colorLabeler.py
│   ├── follow_wall.py
│   └── userPlay.py
├── urdf
│   ├── human.urdf
│   ├── robot2_laser.gazebo
│   ├── robot2_laser.urdf
│   └── robot2_laser.xacro
└── worlds
    └── house2.world
```
*exp_final directory Tree*

TThe main files are stored inside the following directories:

- **scripts**: where all the nodes are stored
- **msg**: where the ROS custom msgs are defined. In particular the the following ones:
	- *BallState.msg*:
		```
			bool BallDetected
			float64 currrentRadious
			string balColor
		```
        Here all the messages related to the ball info are shared via topic *BallState*. 
	- *user.msg*:
		```
			bool play
			string color
		```
        Here all the messages related to the user command are shared via topic *userCommand*.
- **launch**: ROS launch files for starting the simulation
- **param**: files .yaml used to configure the gmapping and the Navigation Stack
- **urdf**: file with the description of the element spawned in the enviroment

The other directories contain file for build the ROS workspaces and the documentation.

## Installation and Running Procedure

As mentioned in the introduction this architecture is based on the ROS Navigation Stack and the gmapping, so please install:
For the openslam_gmapping:

```
user@hostanme$ sudo apt-get install ros-<ros_distro>-openslam-gmapping
```
For the sparse library:

```
user@hostanme$ sudo apt-get install libsuitesparse-dev
```
For the ROS Navigation Stack:
```
user@hostanme$ sudo apt-get install ros-<ros_distro>-navigation
```

Then in order to configure the ROS environment inside the current workspace the commands `catkin_make` is required as well as the sourcing of the setup.*sh files. So, finally, to start the simulation please run:

```
user@hostname$ roslaunch exp_final simulation.launch
```

You’ll notice that two new shell windows will appear after the execution of the launch. One is the output of the node *cmd_man.py* and the other one is the *user interface*.

## System Features
![robot](/assets/robot.png)
*Fig.5: Robot Model*

The model of the enviroment is a simple wheeled robot (two actuated wheels and a castor one) equipped with an Hokuyo range finders and a RGB camera. The model itself is described in the xacro file [robot2_laser.xacro](https://github.com/Sabrin0/exp_final/blob/main/urdf/robot2_laser.xacro) while in the [robot2_laser.gazebo](https://github.com/Sabrin0/exp_final/blob/main/urdf/robot2_laser.gazebo) all the parameters related to the configuration of the sensor are defined.

## System Limitation and Possible Solutions

One of the main limitation of the system is the mobility during the state find. I decided to recall and improve the navigation proposed by the bug algorithm despite of using the [explore-lite pakage](https://github.com/Sabrin0/exp_final/blob/main/urdf/robot2_laser.xacro). Unlike this last one the current implementation is not based on a frontier-based exploration but instead the robot simply follows the wall. For this reason the motion is very slow and there is the high-risk that the robot gets stuck when it try to go though a narrow door. Aside from that, in this way there is the certain that the robot will found all the ball in the environment. As said before, a possible solution is implement a frontier-based exploration with the obstacle avoidance asset.
Another problem is related to the openCV algorithm. Since it detects all the circles in the processed image there is the possibility that there is a false-positive. This problem can be resolved with an update of the dictionary related to the color detection, by enriching it whit more colors. In this way the robot will know in advance which ball track and which do not. Moreover during the track phase, the obstacle avoidance is not very effective since the openCV algorithm directly control the topic *cmd_vel* (responsible of the wheels actuation).

## Author & Contact

Luca Covizzi

luca@francocovizzi.it
