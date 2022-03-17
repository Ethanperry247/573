#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan

# Globals and callback are used for closed loop system
x = 0

Qtable = [

]

def determine_state():
    pass

def turn(angle):
    pass

def move(distance):
    pass

def move_forward():
    pass

def turn_left():
    pass

def turn_right():
    pass

def callback(scan):
    global x
    x = scan
    rospy.loginfo("Done with step 1")

def reinforcement():
    pub = rospy.Publisher('/triton/vel_cmd', Pose2D, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, callback)

    rospy.init_node('reinforcement', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
    
        pose = Pose2D()
        pose.x = 5
        pose.y = 0
        pose.theta = 0
        # rospy.loginfo("Done with step 1")

        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        reinforcement()
    except rospy.ROSInterruptException:
        pass