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
Qt1 = [
            1,
            1,
            2,
            1,
            1,
            2,
            1,
            1,
            2,
            0,
            1,
            2,
            0,
            1,
            2,
            0,
            1,
            2,
            0,
            1,
            2,
            0,
            1,
            2,
            0,
            1,
            2
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
    move(0, -0.3, 0)

def turn_left():
    move(0, 0, -0.3)

def turn_right():
    move(0, 0, 0.3)

def create_average_measurement(scan, arr):
    avg = 0
    for index in arr:
        avg += scan.ranges[index]

    return (avg / len(arr))

def callback(scan):

    # Determine the state of each of these sides based on a threshold
    determine_state(create_average_measurement(scan, LEFT_SCAN_INDICES), 0, 0.25, 1.5)
    determine_state(create_average_measurement(scan, RIGHT_SCAN_INDICES), 2, 0.25, 1.5)
    determine_state(create_average_measurement(scan, FRONT_SCAN_INDICES), 1, 0.25, 1.5)

# Uses thresholds to determine whether the robot is near or far from any given wall
# The close and far boundaries can be passed as params
def determine_state(distance, region, close_boundary, far_boundary):
    global left, front, right
    determination = Distance.MEDIUM

    if (distance < close_boundary):
        determination = Distance.CLOSE
    elif (distance > far_boundary):
        determination = Distance.FAR

    if (region == 0):
        left = determination
    elif (region == 1):
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

        global Qt1

        next_state = Qt1[int(left.value * 9 + front.value * 3 + right.value)]


        if (next_state == 0):
            turn_left()

        if (next_state == 1):
            move_forward()

        if (next_state == 2):
                turn_right()
        
        rate.sleep()

if __name__ == '__main__':
    try:
        reinforcement()
    except rospy.ROSInterruptException:
        pass