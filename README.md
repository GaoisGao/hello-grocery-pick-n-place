# hello-grocery-pick-n-place

A robotics project for handling groceries in a home environment.

## 1. Overview

This project implements a ROS 2-based grocery sorting robot using perception, MoveIt motion planning, and a custom gripper control node. The system detects grocery objects, estimates their 3D positions, plans UR3e arm trajectories, and executes pick-and-place motions.

## 2. System Architecture

The project follows the software pipeline below:

Camera → Perception Node → Object Pose Estimation → MoveIt Planning Node → UR3e Arm Driver → Gripper Control

## 3. Package Structure



## 4. Release Notes

This project was developed with assistance from AI tools, but the overall system architecture and coding process were supervised by the author of the project.

The main components of the project are:

1. An action client that sends target coordinates to MoveIt, allowing the UR3e arm to move to the observation pose and to the predefined placement poses for different storage bins.

2. A camera perception node that subscribes to the RealSense camera topic and runs a YOLOv11 model for object detection. The node estimates the detected object's position in the camera frame, applies a TF transformation, and publishes the object's position relative to the arm's `base_link` frame.

3. A supervisory node that classifies detected object labels based on their storage requirements and assigns each object to the corresponding storage location in the fridge inventory.

A simulated version of this project was completed in collaboration with Alex G. Kautz and Pascale O. Leone for CS 141: Probabilistic Robotics, Spring 2026. In that project, we created a pick-up task in a simulation environment and ran experiments by randomizing the object location and the UR3e arm's tolerance values to evaluate grasp success rate.

The code in this repository is designed for running the ROS project on a real robot and was written independently from the code developed for the CS 141 project deliverables.
