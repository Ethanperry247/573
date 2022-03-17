#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from enum import Enum

class Distance(Enum):
    CLOSE = 0
    MEDIUM = 1
    FAR = 2

class Region(Enum):
    LEFT = 0
    FRONT = 1
    RIGHT = 2


# Globals and callback are used for scanning system
front = Distance.MEDIUM
left = Distance.MEDIUM
right = Distance.MEDIUM

Qtable = [

]


def turn(angle):
    pub = rospy.Publisher("/triton/vel_cmd", Pose2D, queue_size=2)

    pose = Pose2D()
    pose.x = 0.3
    pose.y = 0
    pose.theta = angle

    pub.publish(pose)

def move(speed):
    pub = rospy.Publisher("/triton/vel_cmd", Pose2D, queue_size=2)

    pose = Pose2D()
    pose.x = speed
    pose.y = 0
    pose.theta = 0

    pub.publish(pose)



def move_forward():
    pass

def turn_left():
    pass

def turn_right():
    pass

def callback(scan):
    length = len(scan.ranges) - 1
    avg_left = (scan.ranges[0] + scan.ranges[1] + scan.ranges[2]) / 3
    avg_right = (scan.ranges[length - 1] + scan.ranges[length - 2] + scan.ranges[length - 3]) / 3
    avg_front = (scan.ranges[length // 2] + scan.ranges[(length // 2) - 1] + scan.ranges[(length // 2) + 1]) / 3

    determine_state(avg_left, Region.LEFT)
    determine_state(avg_right, Region.RIGHT)
    determine_state(avg_front, Region.FRONT)

    global left, front, right

    rospy.loginfo(left)
    rospy.loginfo(front)
    rospy.loginfo(right)

def determine_state(distance, region):
    global left, front, right
    determination = Distance.MEDIUM

    if (distance < 0.75):
        determination = Distance.CLOSE
    elif (distance > 2):
        determination = Distance.FAR

    if (region == Region.LEFT):
        left = determination
    elif (region == region.FRONT):
        front = determination
    else:
        right = determination


    

def reinforcement():
    sub = rospy.Subscriber('/scan', LaserScan, callback)

    rospy.init_node('reinforcement', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
    
        # pose = Pose2D()
        # pose.x = 5
        # pose.y = 20
        # pose.theta = 0
        # # rospy.loginfo("Done with step 1")

        # pub.publish(pose)
        move(0.3)
        rate.sleep()

if __name__ == '__main__':
    try:
        reinforcement()
    except rospy.ROSInterruptException:
        pass