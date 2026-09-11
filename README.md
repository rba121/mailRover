MailRover — Autonomous Mail Delivery Robot

MailRover is an autonomous ground robot designed for door-to-door parcel delivery within a building. Developed as an engineering capstone project, the system combines ROS 2, Raspberry Pi 5, 2D LiDAR, autonomous navigation, embedded motor control, and a web-based delivery interface.

A user creates a delivery task through a web interface. The robot autonomously navigates from its loading station to the destination, waits for the recipient to retrieve the parcel using a PIN-verified drawer, and then returns to the loading station for the next delivery.

Overview

MailRover targets autonomous parcel delivery within a single floor of a building, such as a residence or dormitory.

The overall delivery workflow is:

Loading Station
      ↓
Receive Delivery Task
      ↓
Autonomous Navigation
      ↓
Destination / Room
      ↓
Wait for Parcel Pickup
      ↓
PIN-Verified Drawer
      ↓
Return to Loading Station
Core Capabilities
SLAM-generated map of a multi-hundred-square-meter building floor
Autonomous point-to-point navigation
LiDAR-based localization and obstacle avoidance
Encoder-based wheel odometry
PID-based motor control
Web-based delivery task creation and dispatch
Autonomous return-to-loading-station behavior
Physical emergency stop with software-aware pause/resume
PIN-verified parcel retrieval
My Contribution — Navigation & Autonomous Control

I was responsible for the Navigation & Autonomous Control subsystem of MailRover.

My work focused on enabling the physical robot to determine its position, plan paths through the environment, avoid obstacles, and execute navigation commands through the drive system.

Navigation responsibilities
Developed encoder-based wheel odometry
Implemented PID-based motor velocity control
Integrated the SLLidar S2 2D LiDAR with ROS 2
Built and tested the SLAM mapping workflow
Configured and integrated the Nav2 navigation stack
Configured AMCL for localization against the generated map
Integrated and tuned the MPPI controller for path following
Configured global and local costmaps
Implemented collision monitoring and navigation safety behavior
Integrated navigation with the overall delivery workflow
Debugged localization and odometry drift on the physical robot
Tuned navigation parameters through repeated real-world testing

Running the Project

Requirements
Raspberry Pi 5
Ubuntu
ROS 2 Jazzy
Nav2
SLAM / localization packages
SLLidar ROS 2 driver
Wheel encoders
Motor controller
Build the ROS 2 Workspace
cd ~/ros2_ws
colcon build
source install/setup.bash
Launch

Example:

ros2 launch <navigation_package> <navigation_launch_file>.launch.py

Launch files, package names and configuration parameters depend on the specific robot setup.

Technologies

Robotics: ROS 2, Nav2, AMCL, SLAM, TF2
Programming: Python, C/C++
Embedded: Raspberry Pi 5
Sensors: SLLidar S2, Wheel Encoders
Navigation: MPPI, Costmaps, Path Planning, Collision Monitoring
Web: Flask
Operating System: Ubuntu Linux
Tools: RViz2, Git, Linux

