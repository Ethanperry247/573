#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
import math
from enum import Enum

# Distance enum for the three kinds of distance measurements which can be taken
class Distance(Enum):
    CLOSE = 0
    MEDIUM = 1
    FAR = 2

# Region enum for the left, front, and right scanners
class Region(Enum):
    LEFT = 0
    FRONT = 1
    RIGHT = 2

# Action enum for the three actions which may be taken from any state
class Action(Enum):
    TURN_LEFT = 0
    MOVE_FORWARD = 0
    TURN_RIGHT = 0

# Globals and callback are used for scanning system
front = Distance.MEDIUM
left = Distance.MEDIUM
right = Distance.MEDIUM

# These three arrays specify the sensor measurement angles on the lidar
FRONT_SCAN_INDICES = [356, 357, 358, 359, 0, 1, 2, 3]
RIGHT_SCAN_INDICES = [86, 87, 88, 89, 90, 91, 92, 93]
LEFT_SCAN_INDICES = [266, 267, 268, 269, 270, 271, 272, 273]

# 27 states for the 3 distances ** 3 regions of view
# This Q-table describes which actions should be taken at each state to move along a wall
# The states dictate that a robot should try to stay medium distance from its right wall
# These states can be customized to perform different behaviors as well
Qtable = [
                                        # Region - Distance Pairs
    [
        [
            Action.MOVE_FORWARD,        # LC FC RC
            Action.MOVE_FORWARD,        # LC FC RM
            Action.TURN_RIGHT,          # LC FC RF
        ],
        [
            Action.MOVE_FORWARD,        # LC FM RC -- Stay between walls in this instance
            Action.MOVE_FORWARD,        # LC FM RM
            Action.TURN_RIGHT,          # LC FM RF
        ],
        [
            Action.MOVE_FORWARD,        # LC FF RC -- Stay between walls in this instance
            Action.MOVE_FORWARD,        # LC FF RM
            Action.TURN_RIGHT,          # LC FF RF
        ]
    ],
    [
        [
            Action.TURN_LEFT,           # LM FC RC
            Action.MOVE_FORWARD,        # LM FC RM
            Action.TURN_RIGHT,          # LM FC RF
        ],
        [
            Action.TURN_LEFT,           # LM FM RC
            Action.MOVE_FORWARD,        # LM FM RM
            Action.TURN_RIGHT,          # LM FM RF
        ],
        [
            Action.TURN_LEFT,           # LM FF RC
            Action.MOVE_FORWARD,        # LM FF RM
            Action.TURN_RIGHT,          # LM FF RF
        ]
    ],
    [    
        [
            Action.TURN_LEFT,           # LF FC RC
            Action.MOVE_FORWARD,        # LF FC RM
            Action.TURN_RIGHT,          # LF FC RF
        ],
        [
            Action.TURN_LEFT,           # LF FM RC
            Action.MOVE_FORWARD,        # LF FM RM
            Action.TURN_RIGHT,          # LF FM RF
        ],
        [
            Action.TURN_LEFT,           # LF FF RC
            Action.MOVE_FORWARD,        # LF FF RM
            Action.TURN_RIGHT           # LF FF RF
        ] 
    ]
]

# Move publishes a linear and angular direction to the robot
# Only the X is currently used to navigate the robot
def move(angle, x, y):
    pub = rospy.Publisher("/triton_lidar/vel_cmd", Pose2D, queue_size=2)

    pose = Pose2D()
    pose.x = x
    pose.y = y
    pose.theta = angle

    pub.publish(pose)


# Moving forward, to the left, and to the right as according to the project slides
def move_forward():
    move(0, 0.3, 0)

def turn_left():
    move(math.pi / 8, 0.3, 0)

def turn_right():
    move(-math.pi / 8, 0.3, 0)

def create_average_measurement(scan, arr):
    avg = 0
    for index in arr:
        avg += scan.ranges[index]

    return (avg / len(arr))

def callback(scan):

    # Determine the state of each of these sides based on a threshold
    determine_state(create_average_measurement(scan, LEFT_SCAN_INDICES), Region.LEFT, 0.75, 2)
    determine_state(create_average_measurement(scan, RIGHT_SCAN_INDICES), Region.RIGHT, 0.75, 2)
    determine_state(create_average_measurement(scan, FRONT_SCAN_INDICES), Region.FRONT, 0.75, 2)

# Uses thresholds to determine whether the robot is near or far from any given wall
# The close and far boundaries can be passed as params
def determine_state(distance, region, close_boundary, far_boundary):
    global left, front, right
    determination = Distance.MEDIUM

    if (distance < close_boundary):
        determination = Distance.CLOSE
    elif (distance > far_boundary):
        determination = Distance.FAR

    if (region == Region.LEFT):
        left = determination
    elif (region == region.FRONT):
        front = determination
    else:
        right = determination


    

def reinforcement():
    global left, front, right
    rospy.init_node('reinforcement', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        global left, front, right
        
        rospy.loginfo("Left:")
        rospy.loginfo(left)
        rospy.loginfo("Front:")
        rospy.loginfo(front)
        rospy.loginfo("Right:")
        rospy.loginfo(right)

        
        next_state = Qtable[left.value][front.value][right.value]

        rospy.loginfo("Next State:")
        rospy.loginfo(next_state)

        if (next_state == Action.MOVE_FORWARD):
            move_forward()
        elif (next_state == Action.TURN_LEFT):
            turn_left()
        else:
            turn_right()
    
        rate.sleep()

if __name__ == '__main__':
    try:
        reinforcement()
    except rospy.ROSInterruptException:
        pass
