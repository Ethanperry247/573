#!/usr/bin/env python2

import rospy
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import GetModelState
import math
from enum import Enum
import random

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

TOTAL_STATES = 27
TOTAL_ACTIONS = 3
STATE_DIMENSIONS = [3, 3, 3]

# Equation constants recommended in lecture
EPSILON_0 = 0.9
D = 0.985
ALPHA = 0.2
GAMMA = 0.8

# 27 states for the 3 distances ** 3 regions of view
# This Q-table describes which actions should be taken at each state to move along a wall
# These states can be customized to perform different behaviors as well
# 0 Denots a left move, 1 denotes a forward move, and 2 denotes a right move
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


# Moving forward, to the left, and to the right
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

def greedy_epsilon():
    pass

# Initialize Q table according to a given number of states and actions at each one of those states. 
def initialize_table(states, actions):
    Qt = []
    for i in range(states):
        act = []
        # Fill with random numbers
        for j in range(actions):
            act.append(random.random())
        Qt.append(act)
    return Qt

# Will choose an action based on a distribution of values.
def choose_action(dist):
    total = 0
    for num in dist:
        total = total + num
    uni = random.uniform(0, total)
    inc = 0
    for it, num in enumerate(dist):
        if uni < inc:
            return it
        inc += num
    

def identify_reward():
    global left, front, right
    if (left != 1 or front != 1 or right != 1):
        return -1
    else:
        return 0

def provide_reset_simulation():
    rospy.wait_for_service('/gazebo/reset_world')
    return rospy.ServiceProxy('/gazebo/reset_world', Empty)

def provide_get_robot_state():
    rospy.wait_for_service('/gazebo/get_model_state')
    return rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

def initial_state(table):
    reset = provide_reset_simulation()
    reset()

    return table[int(left.value * STATE_DIMENSIONS[0] * STATE_DIMENSIONS[1] + front.value * STATE_DIMENSIONS[2] + right.value)]



def take_action(action):
    if (action == 0):
        turn_left()

    if (action == 1):
        move_forward()

    if (action == 2):
        turn_right()


def q_learning(episodes):
    global left, front, right
    Qt2 = initialize_table(TOTAL_STATES, TOTAL_ACTIONS)

    # Define robot rate.
    rate = rospy.Rate(10) # 10hz

    # Get pose function for determining termination conditions.
    get_pose = provide_get_robot_state()

    for num, episode in enumerate(episodes):
        terminal_condition = False
        iteration = 0

        # Reset the simulation and get the new initial state.
        state = initial_state(Qt2)

        while not terminal_condition:

            # Use greedy epsilon to determine the distribution of probabilities.
            action_distribution = greedy_epsilon(state)

            # Choose an action at the current state based on the action distribution.
            chosen_action = choose_action_index(action_distribution)

            # Make the robot take an action based on the random choice.
            take_action(chosen_action)

            # Use custom reward function to determine the reward for the action.
            reward = identify_reward()

            # Since state will be updated in globals by the subscriber callback, access the next state in the table.
            next_state = Qt2[int(left.value * STATE_DIMENSIONS[0] * STATE_DIMENSIONS[1] + front.value * STATE_DIMENSIONS[2] + right.value)]

            # Update Q table as according to lecture.
            Qt2[state][chosen_action] = Qt2[state][chosen_action] + ALPHA * (reward + GAMMA * max(next_state) - Qt2[state][chosen_action])

            # If the probability dips beneath 0, then set to 0.
            if (Qt2[state][chosen_action] < 0):
                Qt2[state][chosen_action] = 0

            # Update state.
            state = next_state

            # After 1000 iterations without crashing into the wall, commense the next episode.
            iteration += 1
            if (iteration > 1000):
                terminal_condition = True

            rospy.loginfo(get_pose())

            # Just return if rospy is trying to shut down.
            if rospy.is_shutdown():
                return []

            rate.sleep()        

    return Qt2

def reinforcement():
    global left, front, right
    rospy.init_node('reinforcement', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)

    # 10000 Steps as recommended by the lecture.
    q_learning(10000)

if __name__ == '__main__':
    try:
        reinforcement()
    except rospy.ROSInterruptException:
        pass