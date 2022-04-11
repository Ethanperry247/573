#!/usr/bin/env python2

import rospy
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from enum import Enum
import random
import json
from copy import deepcopy

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
    DIAGONAL = 3

# Action enum for the three actions which may be taken from any state
class Action(Enum):
    TURN_LEFT = 0
    MOVE_FORWARD = 1
    TURN_RIGHT = 2

# Globals and callback which are used for scanning system
front = Distance.MEDIUM
left = Distance.MEDIUM
right = Distance.MEDIUM
diagonal = Distance.MEDIUM
orientation = 0
pose = None
last_scan = None

# Sensor angle measurements for the lidar. They are coupled with their 45 degree offsets for the diagonal scanner.
FRONT_SCAN_INDICES = [[355, 356, 357, 358, 359, 0, 1, 2, 3, 4, 5], [44, 45, 46]]
RIGHT_SCAN_INDICES = [[265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275], [314, 315, 316]]
BACK_SCAN_INDICES = [[175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185], [224, 225, 226]]
LEFT_SCAN_INDICES = [[85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95], [134, 135, 136]]

# 54 Total States, 3 actions per state, and 4 state "dimensions"
TOTAL_STATES = 54
TOTAL_ACTIONS = 3
STATE_DIMENSIONS = [2, 3, 3, 3]

# Equation constants recommended in lecture
EPSILON_0 = 0.9
D_GREEDY = 0.985
ALPHA = 0.2
GAMMA = 0.8
TERMINATION_EPSILON = 0.05

# Move publishes a linear and angular direction to the robot
# Only the X is currently used to navigate the robot
def move(angle, x, y):
    pub = rospy.Publisher("/triton_lidar/vel_cmd", Pose2D, queue_size=2)

    pose = Pose2D()
    pose.x = x
    pose.y = y
    pose.theta = angle

    pub.publish(pose)

# Virtual robot orientation shifting. Part of the action design choices for the robot in order for it to work most effectively with the maze.
def change_virtual_orientation(left):
    global orientation
    if (left and orientation == 0):
        orientation = 3
    elif (not left and orientation == 3):
        orientation = 0
    elif (left):
        orientation -= 1
    else: 
        orientation += 1

# Sensor rotations to accompany the virtual turning.
def rotate_sensors_left():
    change_virtual_orientation(True)
    global FRONT_SCAN_INDICES, RIGHT_SCAN_INDICES, LEFT_SCAN_INDICES, BACK_SCAN_INDICES
    right = deepcopy(RIGHT_SCAN_INDICES)
    RIGHT_SCAN_INDICES = deepcopy(FRONT_SCAN_INDICES)
    FRONT_SCAN_INDICES = deepcopy(LEFT_SCAN_INDICES)
    LEFT_SCAN_INDICES = deepcopy(BACK_SCAN_INDICES)
    BACK_SCAN_INDICES = right

def rotate_sensors_right():
    change_virtual_orientation(False)
    global FRONT_SCAN_INDICES, RIGHT_SCAN_INDICES, LEFT_SCAN_INDICES, BACK_SCAN_INDICES
    back = deepcopy(BACK_SCAN_INDICES)
    BACK_SCAN_INDICES = deepcopy(LEFT_SCAN_INDICES) 
    LEFT_SCAN_INDICES = deepcopy(FRONT_SCAN_INDICES)
    FRONT_SCAN_INDICES = deepcopy(RIGHT_SCAN_INDICES)
    RIGHT_SCAN_INDICES = back

# Moving forward, to the left, and to the right.
def move_forward():
    global orientation

    if (orientation == 0):
        move(0, 0.5, 0)
    if (orientation == 1):
        move(0, 0, -0.5)
    if (orientation == 2):
        move(0, -0.5, 0)
    if (orientation == 3):
        move(0, 0, 0.5)

def virtual_turn_left():
    rotate_sensors_left()

def virtual_turn_right():
    rotate_sensors_right()

# 
def create_average_measurement(scan, arr):
    avg = 0
    for index in arr:
        avg += scan.ranges[index]
    return (avg / len(arr))

# Callback for 3D scanning.
def callback(scan):
    global FRONT_SCAN_INDICES, RIGHT_SCAN_INDICES, LEFT_SCAN_INDICES, BACK_SCAN_INDICES
    global last_scan
    last_scan = scan

    # Determine the state of each of these sides based on a threshold
    determine_state(create_average_measurement(scan, LEFT_SCAN_INDICES[0]), 0, 0.5, 0.7)
    determine_state(create_average_measurement(scan, FRONT_SCAN_INDICES[0]), 1, 0.5, 0.7)
    determine_state(create_average_measurement(scan, RIGHT_SCAN_INDICES[0]), 2, 0.5, 0.7)
    determine_state(create_average_measurement(scan, BACK_SCAN_INDICES[1]), 3, 0.9, 0.9)

# Uses thresholds to determine whether the robot is near or far from any given wall
# The close and far boundaries can be passed as params
def determine_state(distance, region, close_boundary, far_boundary):
    global left, front, right, diagonal
    determination = Distance.MEDIUM

    if (distance < close_boundary):
        determination = Distance.CLOSE
    elif (distance < far_boundary):
        determination = Distance.MEDIUM
    elif (distance > far_boundary):
        determination = Distance.FAR

    if (region == 0):
        left = determination
    elif (region == 1):
        front = determination
    elif (region == 2):
        right = determination
    elif (region == 3):
        if (determination == Distance.CLOSE):
            diagonal = Distance.CLOSE
        else:
            diagonal = Distance.MEDIUM


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
# If zeros is set to false, then the table will be initialized to random numbers, otherwise it will be set to zeros.
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
    global left, front, right, diagonal

    # Indicates the ending of an I shaped turn, in which the robot shouldn't get punished.
    if (right.value == 2 and diagonal.value == 0):
        return 0

    # Minor punishment if robot is not in a medium place from the right wall, if the robot is close to a front wall, or if the robot is close to a left wall.
    if (right.value != 1 or left.value == 0 or front.value == 0):
        return -1
    
    # Otherwise the robot doesn't get a punishment.
    return 0

# Accesses a service to reset the simulation.
def provide_reset_simulation():
    rospy.wait_for_service('/gazebo/reset_world')
    return rospy.ServiceProxy('/gazebo/reset_world', Empty)

# Will randomly begin the robot and provide an appropriate starting state.
def initial_state():
    global left, front, right, diagonal
    reset = provide_reset_simulation()
    reset()

    # Randomly initiate the location of the triton robot.
    msg = ModelState()
    msg.model_name = 'triton_lidar'
    msg.pose.position.x = random.uniform(-3.5, 3.5)
    msg.pose.position.y = random.uniform(-3.5, 3.5)
    msg.pose.position.z = 0

    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    set_state(msg)

    return int(diagonal.value * STATE_DIMENSIONS[1] * STATE_DIMENSIONS[2] * STATE_DIMENSIONS[3] + left.value * STATE_DIMENSIONS[2] * STATE_DIMENSIONS[3] + front.value * STATE_DIMENSIONS[3] + right.value)


# Will take a number to execute an action, being either a forward movement or a turn.
def take_action(action):
    if (action == 0):
        virtual_turn_left()

    if (action == 1):
        move_forward()

    if (action == 2):
        virtual_turn_right()

# State times are the poses which the collision condition checks across.
# The termination epsilon is the mnaximum difference between each time step.
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

# Parameters:
# episodes - determines the number of episodes.
# read_table_from_file - determines whether to create a new table of zeros or read from a labeled file.
# base_d - the base "d" for the greedy epsilon algorithm, which will effect the exploit/explore ratio.
# file_read_number - the number associated with the file to read a table from.
# file_write_number - the number associated with the file to write a working table to.
def q_learning(episodes, read_table_from_file, base_d, file_read_number, file_write_number):
    global left, front, right, diagonal
    global pose
    Qt2 = None
    if (read_table_from_file):
        Qt2 = json.load(open("qt" + str(file_read_number) + ".json", "r"))
    else:
        Qt2 = initialize_table(TOTAL_STATES, TOTAL_ACTIONS, zeros=True)

    # Define robot rate.
    rate = rospy.Rate(8) # 2 - 8 hz tested. 8 seems to work best with this movement scheme.

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
            global last_scan

            action_index = greedy_epsilon(Qt2, state_index, episode + base_d, EPSILON_0, D_GREEDY, TOTAL_ACTIONS)

            # Make the robot take an action based on the random choice.
            take_action(action_index)

            # Enforce the callback to update sensor readings in case the callback isn't quick enough to update on its own.
            if (last_scan is not None):
                callback(last_scan)
            
            # Use custom reward function to determine the reward for the action.
            reward = identify_reward()

            # Since state will be updated in globals by the subscriber callback, access the next state in the table.
            next_state_index = int(diagonal.value * STATE_DIMENSIONS[1] * STATE_DIMENSIONS[2] * STATE_DIMENSIONS[3] + left.value * STATE_DIMENSIONS[2] * STATE_DIMENSIONS[3] + front.value * STATE_DIMENSIONS[3] + right.value)

            # Update Q table as according to lecture.
            # Qt2[state_index][action_index] = (1 - ALPHA) * Qt2[state_index][action_index] + ALPHA * (float(reward) + GAMMA * max(Qt2[next_state_index]))

            # Update state.
            state_index = next_state_index

            # After 1000 - 2000 iterations without crashing into the wall, commense the next episode.
            iteration += 1
            if (iteration > 1500):
                terminal_condition = True

            # Return an empty array if rospy is trying to shut down.
            if rospy.is_shutdown():
                return []
            
            # Record a moving window of every three poses to determine if the robot will crash into a wall or not.
            if (state_time_1 is not None):
                state_time_2 = state_time_1

            if (state_time_0 is not None):
                state_time_1 = state_time_0

            state_time_0 = pose

            # Use the recorded poses to check a collision termination.
            if (check_collision_termination(state_time_0, state_time_1, state_time_2, TERMINATION_EPSILON)):
                terminal_condition = True

            rate.sleep()

        # Save to the JSON file with each episode
        json.dump(Qt2, open("qt" + str(file_write_number) + ".json", "w"))

    return Qt2

# Update the pose according to a callback.
def pose_callback(position):
    global pose
    index = position.name.index('triton_lidar')
    pose = position.pose[index].position

def reinforcement():
    rospy.init_node('reinforcement', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    sub2 = rospy.Subscriber('/gazebo/model_states', ModelStates, pose_callback)

    # 10000 Steps as recommended by the lecture.
    table = q_learning(10000, True, 100000, 100, 101)

if __name__ == '__main__':
    try:
        reinforcement()
    except rospy.ROSInterruptException:
        pass