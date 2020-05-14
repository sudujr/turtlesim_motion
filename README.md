# TurtleSim_Navigation
  A simple Ros Program to Navigate Turtlesim using C++ for Udacity C++ NanoDegree Capstone project. The turtle sim is navigates using user input commands

# How to Run:
The project is in the folder catkin_ws
* `cd catkin_ws`
* `catkin_make`
* `source devel/setup.bash`
* `roslaunch turtlesim_motion motion.launch`

# Inputs for Navigation
-  linear NAvigation -> linear speed , distance and forward or  backward motion
-  Rotational Navigation ->  angular speed, angle in radians , forward or backward motion
-  spiral motion -> linear speed and angular speed


# Program Structure
- Robot_motion.h/Robotmotion.cpp
      -> includes a class called motion which has private data members for publishing and subscribing to turtlesim nodes and public member function to assist in motion of turtlesim
