# Turtlebot3 Obstacle Avoidance

This ROS package implements a control program for Turtlebot3 in Gazebo that keeps the robot moving without colliding with obstacles in the standard World.

## Requirements

- ROS1 Noetic
- Ubuntu 20.04
- Turtlebot3 packages
- Gazebo

## Installation

1. Create a catkin workspace if you haven't already:

bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

2. Clone this repository into the src folder of your catkin workspace:

cd ~/catkin_ws/src
git clone https://github.com/your_username/turtlebot3_obstacle_avoidance.git

3. Install dependencies:

sudo apt-get update
sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-navigation

4. Build the package:

cd ~/catkin_ws
catkin_make

5. Source the setup file:

source ~/catkin_ws/devel/setup.bash

## Usage

1. Launch the Turtlebot3 in Gazebo:

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch

2. Launch the navigation stack:

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

3. Run the obstacle avoidance node:

rosrun turtlebot3_obstacle_avoidance turtlebot3_obstacle_avoidance.py

## Verification

To verify that the program is working correctly:
	1. Observe the Turtlebot3 in Gazebo. It should move between four predefined points (2, 2), (-2, 2), (-2, -2), and (2, -2) while avoiding obstacles.
	2. Use RViz to visualize the robot's path and the obstacles it's avoiding:

roslaunch turtlebot3_navigation turtlebot3_navigation.launch

	3. Check the terminal output for any error messages or unexpected behavior.
	4. You can also use rqt_graph to visualize the ROS node graph and ensure that all necessary nodes are running and communicating properly:
	rqt_graph
	


 
# qibi_turtlebot_obstacle_avoidance
