Name: Ethan Perry
Course Number: CSCI 573
Date: 03/18/2022
Notes:

In order to run this project, please use the following commands within the project directory:

catkin_make
source /opt/ros/melodic/setup.bash
source ~/Stingray-Simulation/catkin_ws/devel/setup.bash
source ~/Stingray-Simulation/stingray_setup.bash
sudo chmod -R -f 777 ./src/reinforcement/scripts/reinforcement.py
roslaunch reinforcement reinforcement.launch

This is assuming that the Stingray simulation is ready and set up on the system in the home directory. If this is not the correct assumption, please let me know at eperry1@mines.edu, and I will be able to make the correction for you. 

The Q table is currently initialized with manually defined values. Feel free to change the values to get a different outcome.