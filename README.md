# Autonomous Vehicle Simulation Package
This repository contains the ROS 2 package for simulating an autonomous vehicle capable of obstacle avoidance, path tracking, edge detection, and object recognition in the NXP-AIM India Simulator.

## Package Structure
b3rb_ros_edge_vectors
Detects track boundaries using OpenCV to keep the vehicle within the lane limits.

b3rb_ros_line_follower
Handles path tracking and obstacle avoidance by processing camera and sensor data.

b3rb_ros_object_recog
Performs stop sign recognition and handles image callbacks for other modules.

Yolov8Model1.py
Contains the YOLOv8 model weights used for stop sign detection.

obstacle_config.json
Stores obstacle locations on the map for obstacle avoidance logic.

Simulator Information
The simulation can be tested using the NXP-AIM India Simulator.

## Installation Guide
Refer to the official NXP AIM India Simulator GitBook for installation and setup.

Note: Ensure the simulator is configured to work with ROS 2 Humble.
## Limitations
Real-Time Factor (RTF): The code was tested at an RTF of 0.4 - 0.6, which may cause unstable behavior.

## Videos

### Without Obstacles  

https://github.com/user-attachments/assets/2b4913b2-b72a-4def-8b55-bb770378dd63



### With Obstacles  

https://github.com/user-attachments/assets/c9af156d-26a6-45d1-8ed0-9ad88110d05d

