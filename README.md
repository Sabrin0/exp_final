![Unige](/assets/Unige.png)
# Experimental Final Assignment
### Table of Contents 
- [Experimental Final Assignment](#experimental-final-assignment)
    - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Software Architecture](#software-architecture)
    - [Nodes](#nodes)
      - [FSM](#fsm)
  - [Package and File List](#package-and-file-list)


## Introduction
![enviroment](/assets/enviroment.png "Enviromen top view")
*Fig. 1: Enviroment Top View*

This project is the further work of the previous [assignment](https://github.com/Sabrin0/Assignment2-Experimental-RoboticS-LAB.git) of *Experimental robotics lab*.
Due to the [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) and the [ROS Navighation Stack](http://wiki.ros.org/navigation) the  wheeled robot is able to move autonomously.  The main purpose of this project is to implement a FSM which allows the robot to learn the environment and interact with the user in order to reach the desired room. Each colored ball represents a different room (as shown below) and at the beginning of the simulation all the locations are unknown. It’s up to the robot to navigate and store the informations about the environment though a simple exploring algorithm.

## Software Architecture

### Description
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
#### FSM
![FSM diagram](/assets/FSM.png)

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
