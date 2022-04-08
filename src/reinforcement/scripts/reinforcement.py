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
pose = None

# These three arrays specify the sensor measurement angles on the lidar
FRONT_SCAN_INDICES = [355, 356, 357, 358, 359, 0, 1, 2, 3, 4]
RIGHT_SCAN_INDICES = [85, 86, 87, 88, 89, 90, 91, 92, 93, 94]
LEFT_SCAN_INDICES = [265, 266, 267, 268, 269, 270, 271, 272, 273, 274]

TOTAL_STATES = 125
TOTAL_ACTIONS = 3
STATE_DIMENSIONS = [5, 5, 5]

# Equation constants recommended in lecture
EPSILON_0 = 0.9
D_GREEDY = 0.985
ALPHA = 0.2
GAMMA = 0.9
TERMINATION_EPSILON = 0.01

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
    # move(math.pi / 20, -0.3, -0.3)
    move(math.pi / 2, -0.2, 0)
    # move(0, 0, -0.3)

def turn_right():
    # move(- math.pi / 20, -0.3, 0.3)
    move(-math.pi / 2, -0.2, 0)
    # move(0, 0, 0.3)

def create_average_measurement(scan, arr):
    avg = 0
    for index in arr:
        avg += scan.ranges[index]

    return (avg / len(arr))

def callback(scan):

    # Determine the state of each of these sides based on a threshold
    determine_state(create_average_measurement(scan, LEFT_SCAN_INDICES), 0, 0.35, 0.6)
    determine_state(create_average_measurement(scan, RIGHT_SCAN_INDICES), 2, 0.35, 0.6)
    determine_state(create_average_measurement(scan, FRONT_SCAN_INDICES), 1, 0.35, 0.6)

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

def greedy_epsilon(table, state_index, episode, epsilon_0, d, num_actions):
    # Epsilon equation recommended by slides.
    epsilon = epsilon_0 * (d ** episode)
    rand = random.random()

    state = table[state_index]

    # Check if rand is bigger. More exploration will occur in earlier episodes.
    if (rand > epsilon):
        return state.index(max(state)) 
    else:
        return random.randint(0, num_actions - 1)

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
    if (left.value == 0 or right.value != 1 or front.value == 0):
        # print("Bad")
        return -1
    else:
        print("Good")
        return 1

def provide_reset_simulation():
    rospy.wait_for_service('/gazebo/reset_world')
    return rospy.ServiceProxy('/gazebo/reset_world', Empty)

def initial_state():
    global left, front, right
    reset = provide_reset_simulation()
    reset()

    msg = ModelState()
    msg.model_name = 'triton_lidar'
    msg.pose.position.x = random.uniform(-3.5, 3.5)
    msg.pose.position.y = random.uniform(-3.5, 3.5)
    msg.pose.position.z = 0
    # msg.twist.angular.x = random.uniform(0, math.pi * 2)

    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    set_state(msg)

    return int(left.value * STATE_DIMENSIONS[0] * STATE_DIMENSIONS[1] + front.value * STATE_DIMENSIONS[2] + right.value)



def take_action(action):
    if (action == 0):
        turn_left()

    if (action == 1):
        move_forward()

    if (action == 2):
        turn_right()

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


def q_learning(episodes, read_table_from_file):
    global left, front, right
    global pose
    Qt2 = None
    if (read_table_from_file):
        Qt2 = json.load(open("qt3.json", "r"))
    else:
        Qt2 = initialize_table(TOTAL_STATES, TOTAL_ACTIONS)

    # Define robot rate.
    rate = rospy.Rate(8) # 12hz

    # Get pose function for determining termination conditions.
    # get_pose = provide_get_robot_state()

    for episode in range(episodes):
        terminal_condition = False
        iteration = 0
        print(episode)

        # Reset the simulation and get the new initial state.
        state_index = initial_state()

        state_time_0 = None
        state_time_1 = None
        state_time_2 = None

        while not terminal_condition:

            # Use greedy epsilon to determine the distribution of probabilities.
            # action_distribution = greedy_epsilon(state)

            # Choose an action at the current state based on the action distribution.
            # chosen_action = choose_action_index(action_distribution)

            action_index = greedy_epsilon(Qt2, state_index, episode, EPSILON_0, D_GREEDY, TOTAL_ACTIONS)

            # Make the robot take an action based on the random choice.
            take_action(action_index)

            # Use custom reward function to determine the reward for the action.
            reward = identify_reward()

            # Since state will be updated in globals by the subscriber callback, access the next state in the table.
            next_state = int(left.value * STATE_DIMENSIONS[0] * STATE_DIMENSIONS[1] + front.value * STATE_DIMENSIONS[2] + right.value)

            # Update Q table as according to lecture.
            Qt2[state_index][action_index] = Qt2[state_index][action_index] + ALPHA * (reward + GAMMA * max(Qt2[next_state]) - Qt2[state_index][action_index])

            # If the probability dips beneath 0, then set to 0.
            # if (Qt2[state_index][action_index] < 0):
            #     Qt2[state_index][action_index] = 0

            # Update state.
            state = next_state

            # After 1000 iterations without crashing into the wall, commense the next episode.
            iteration += 1
            if (iteration > 1000):
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

        
        json.dump(Qt2, open("qt3.json", "w"))

    return Qt2

def pose_callback(position):
    global pose
    index = position.name.index('triton_lidar')
    pose = position.pose[index].position

def reinforcement():
    global left, front, right
    rospy.init_node('reinforcement', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    sub2 = rospy.Subscriber('/gazebo/model_states', ModelStates, pose_callback)

    # 10000 Steps as recommended by the lecture.
    table = q_learning(10000, False)

    # rate = rospy.Rate(10) # 10hz
    # initial_state()

    # state_time_0 = None
    # state_time_1 = None
    # state_time_2 = None
    # termination_epsilon = 0.0001


    #  #Determine the next state and execute the appropriate action to get to that state. 
    # while not rospy.is_shutdown():
    #     global Qt1
    #     global pose
    #     # Used to get the correct state.
    #     next_state = Qt1[int(left.value * 9 + front.value * 3 + right.value)]
    #     if (next_state == 0):
    #         turn_left()
    #     if (next_state == 1):
    #         move_forward()
    #     if (next_state == 2):
    #         turn_right()

        
        
    #     rate.sleep()

if __name__ == '__main__':
    try:
        reinforcement()
    except rospy.ROSInterruptException:
        pass