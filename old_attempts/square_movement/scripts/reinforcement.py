#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
import math
from enum import Enum
import random
import json

# # Distance enum for the three kinds of distance measurements which can be taken
class Distance(Enum):
    CLOSE = 0
    MEDIUM = 1
    FAR = 2

# Region enum for the left, front, and right scanners
class Region(Enum):
    LEFT = 0
    FRONT = 1
    RIGHT = 2
    BACK = 4

# Action enum for the three actions which may be taken from any state
class Action(Enum):
    TURN_LEFT = 0
    MOVE_FORWARD = 1
    TURN_RIGHT = 2

# Globals and callback are used for scanning system
front = Distance.MEDIUM
left = Distance.MEDIUM
right = Distance.MEDIUM
back = Distance.MEDIUM
diagonal_front_left = Distance.MEDIUM
diagonal_front_right = Distance.MEDIUM
diagonal_back_left = Distance.MEDIUM
diagonal_back_right = Distance.MEDIUM
pose = None

# These three arrays specify the sensor measurement angles on the lidar
FRONT_SCAN_INDICES = [42, 43, 44, 45, 46, 47, 48]
RIGHT_SCAN_INDICES = [312, 313, 314, 315, 316, 317, 318]
LEFT_SCAN_INDICES = [132, 133, 134, 135, 136, 137, 138]
BACK_SCAN_INDICES = [222, 223, 224, 225, 226, 227, 228]

TOTAL_STATES = 16
TOTAL_ACTIONS = 4
STATE_DIMENSIONS = [2, 2, 2, 2]

# Equation constants recommended in lecture
EPSILON_0 = 0.9
D_GREEDY = 0.985
ALPHA = 0.2
GAMMA = 0.8
TERMINATION_EPSILON = 0.05

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
    move(0, 0.5, 0)

def turn_left():
    move(0, 0, 0.5)

def turn_right():
    move(0, 0, -0.5)

def move_back():
    move(0, -0.5, 0)

def create_average_measurement(scan, arr):
    avg = 0
    for index in arr:
        avg += scan.ranges[index]

    return (avg / len(arr))

def callback(scan):

    # Determine the state of each of these sides based on a threshold
    determine_state(create_average_measurement(scan, LEFT_SCAN_INDICES), 0, 0.4, 0.6)
    determine_state(create_average_measurement(scan, FRONT_SCAN_INDICES), 1, 0.4, 0.6)
    determine_state(create_average_measurement(scan, RIGHT_SCAN_INDICES), 2, 0.4, 0.6)
    determine_state(create_average_measurement(scan, BACK_SCAN_INDICES), 3, 0.4, 0.6)

# Uses thresholds to determine whether the robot is near or far from any given wall
# The close and far boundaries can be passed as params
def determine_state(distance, region, close_boundary, far_boundary):
    global left, front, right, back
    determination = Distance.MEDIUM
    
    # if (distance < close_boundary):
    #     determination = Distance.CLOSE
    if (distance < far_boundary):
        determination = Distance.CLOSE
    elif (distance > far_boundary):
        determination = Distance.MEDIUM

    # if (distance < close_boundary):
    #     determination = Distance.CLOSE
    # elif (distance < far_boundary):
    #     determination = Distance.MEDIUM
    # elif (distance > far_boundary):
    #     determination = Distance.FAR

    if (region == 0):
        left = determination
    elif (region == 1):
        front = determination
    elif (region == 2):
        right = determination
    else:
        back = determination

def greedy_epsilon(table, state_index, episode, epsilon_0, d, num_actions):
    # Epsilon equation recommended by slides.
    epsilon = epsilon_0 * (d ** episode)
    rand = random.random()

    state = table[state_index]

    # Check if rand is bigger. More exploration will occur in earlier episodes.
    if (rand > epsilon):
        print("Exploit")
        return state.index(max(state)) 
    else:
        print("Explore")
        return random.randint(0, num_actions - 1)

# Initialize Q table according to a given number of states and actions at each one of those states. 
def initialize_table(states, actions, zeros):
    Qt = []
    for i in range(states):
        act = []
        # Fill with random numbers
        for j in range(actions):
            if (zeros):
                act.append(0)
            else:
                act.append(random.random())
        Qt.append(act)
    return Qt

def identify_reward():
    global left, front, right, back
    # Minor punishment if robot is not in a medium place from the right wall, if the robot is close to a front wall, or if the robot is close to a left wall.

    if (right.value == 0):
        return 0

    if (right.value != 1 or left.value == 0 or front.value == 0 or back.value == 0):
        # print("Bad")
        return -1
    # Otherwise the robot doesn't get a punishment.
    else:
        # print("Good Robo")
        return 0

def provide_reset_simulation():
    rospy.wait_for_service('/gazebo/reset_world')
    return rospy.ServiceProxy('/gazebo/reset_world', Empty)

def initial_state():
    global left, front, right, back
    reset = provide_reset_simulation()
    reset()

    # Randomly initiate the location of the triton robot.
    msg = ModelState()
    msg.model_name = 'triton_lidar'
    msg.pose.position.x = random.uniform(-3.5, 3.5)
    msg.pose.position.y = random.uniform(-3.5, 3.5)
    # msg.pose.position.x = 1.5
    # msg.pose.position.y = -3.5
    msg.pose.position.z = 0

    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    set_state(msg)

    return int(left.value * 8 + front.value * 4 + right.value * 2 + back.value)



def take_action(action):
    if (action == 0):
        turn_left()

    if (action == 1):
        move_forward()

    if (action == 2):
        turn_right()

    if (action == 3):
        move_back()

def check_collision_termination(state_time_0, state_time_1, state_time_2, termination_epsilon):
    if (state_time_0 is not None and state_time_1 is not None and state_time_2 is not None):
        state_time_0_abs_x = abs(state_time_0.x)
        state_time_1_abs_x = abs(state_time_1.x)
        state_time_2_abs_x = abs(state_time_2.x)
        state_time_0_abs_y = abs(state_time_0.y)
        state_time_1_abs_y = abs(state_time_1.y)
        state_time_2_abs_y = abs(state_time_2.y)

        if (state_time_0_abs_x < state_time_1_abs_x + termination_epsilon and state_time_0_abs_x > state_time_1_abs_x - termination_epsilon):
            if (state_time_0_abs_x < state_time_2_abs_x + termination_epsilon and state_time_0_abs_x > state_time_2_abs_x - termination_epsilon):
                if (state_time_1_abs_x < state_time_2_abs_x + termination_epsilon and state_time_1_abs_x > state_time_2_abs_x - termination_epsilon):
                    if (state_time_0_abs_y < state_time_1_abs_y + termination_epsilon and state_time_0_abs_y > state_time_1_abs_y - termination_epsilon):
                        if (state_time_0_abs_y < state_time_2_abs_y + termination_epsilon and state_time_0_abs_y > state_time_2_abs_y - termination_epsilon):
                            if (state_time_1_abs_y < state_time_2_abs_y + termination_epsilon and state_time_1_abs_y > state_time_2_abs_y - termination_epsilon):
                                return True

    return False


def q_learning(episodes, read_table_from_file, base_d):
    global left, front, right, back
    global pose
    Qt2 = None
    if (read_table_from_file):
        Qt2 = json.load(open("qt20.json", "r"))
    else:
        Qt2 = initialize_table(TOTAL_STATES, TOTAL_ACTIONS, zeros=True)
        json.dump(Qt2, open("qt30.json", "w"))

    # Define robot rate.
    rate = rospy.Rate(2) # 2hz

    for episode in range(episodes):
        terminal_condition = False
        iteration = 0
        print("Running episode: " + str(episode))

        # Reset the simulation and get the new initial state.
        state_index = initial_state()

        state_time_0 = None
        state_time_1 = None
        state_time_2 = None

        while not terminal_condition:

            action_index = greedy_epsilon(Qt2, state_index, episode + base_d, EPSILON_0, D_GREEDY, TOTAL_ACTIONS)

            print("Next Action: "  + str(action_index))

            # Make the robot take an action based on the random choice.
            take_action(action_index)

            # Use custom reward function to determine the reward for the action.
            reward = identify_reward()

            # Since state will be updated in globals by the subscriber callback, access the next state in the table.
            next_state = int(left.value * 8 + front.value * 4 + right.value * 2 + back.value)

            print("State Index: " + str(next_state))

            # Update Q table as according to lecture.
            Qt2[state_index][action_index] = (1 - ALPHA) * Qt2[state_index][action_index] + ALPHA * (float(reward) + GAMMA * max(Qt2[next_state]))

            # Update state.
            state_index = next_state

            # After 1000 iterations without crashing into the wall, commense the next episode.
            iteration += 1
            if (iteration > 800):
                terminal_condition = True

            # Just return if rospy is trying to shut down.
            if rospy.is_shutdown():
                return []
            
            if (state_time_1 is not None):
                state_time_2 = state_time_1

            if (state_time_0 is not None):
                state_time_1 = state_time_0

            state_time_0 = pose

            if (check_collision_termination(state_time_0, state_time_1, state_time_2, TERMINATION_EPSILON)):
                terminal_condition = True

            rate.sleep()
        rate.sleep()
        json.dump(Qt2, open("qt20.json", "w"))

    return Qt2

def pose_callback(position):
    global pose
    index = position.name.index('triton_lidar')
    pose = position.pose[index].position

def reinforcement():
    rospy.init_node('reinforcement', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    sub2 = rospy.Subscriber('/gazebo/model_states', ModelStates, pose_callback)

    # 10000 Steps as recommended by the lecture.
    # table = q_learning(10000, False, 0)

    Qt1 = json.load(open("qt31.json", "r"))

    rate = rospy.Rate(1) # 10hz
    initial_state()

    # Determine the next state and execute the appropriate action to get to that state. 
    while not rospy.is_shutdown():
        global left, front, right, back

        q_table_index = int(left.value * 8 + front.value * 4 + right.value * 2 + back.value)

        # Used to get the correct state.
        next_state = Qt1[q_table_index]
        state_index = next_state.index(max(next_state))
        
        print("Left:    " + str(left) + "   Front:   " + str(front)+ "  Right:   " + str(right) + "  Back:   " + str(back) + "     Next State: " + str(state_index) + "    State Index: " + str(q_table_index))
        print("State Index: " + str(next_state))
        if (state_index == 0):
            turn_left()
        if (state_index == 1):
            move_forward()
        if (state_index == 2):
            turn_right()
        if (state_index == 3):
            move_back()

        rate.sleep()

if __name__ == '__main__':
    try:
        reinforcement()
    except rospy.ROSInterruptException:
        pass