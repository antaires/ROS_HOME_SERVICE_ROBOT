# Home Service Robot

A project that creates a robot to autonomously map an environment, receive and navigate to locations and pick up and deliver a package to different points in the environment. 

## Project Setup

```
$sudo apt-get update
$mkdir -p catkin_ws/src
$cd catkin_ws/src
$catkin_init_workspace
$cd ../
$catkin_make
```
if any required folders are empty, please clone required repos into the source folder: 
```
$ cd catkin_ws/src
$ git clone https://github.com/ros-perception/slam_gmapping.git
$ git clone https://github.com/turtlebot/turtlebot.git
$ git clone https://github.com/turtlebot/turtlebot_interactions.git
$ git clone https://github.com/turtlebot/turtlebot_simulator.git
```
intall package dependencies with 
```
$ rosdep install [package_name]
```
copy the remaining contents of the src folder from this repo.

## To Run 
```
$ cd catkin_ws/src/scripts
$ chmod +x home_service.sh
$ ./home_service.sh
```
The robot should appear in Gazebo, and then in RViz where you will see the robot conduct path planning to reach a pickup point (blue marker) then transport the marker to the dropoff point, where it appears again. 
