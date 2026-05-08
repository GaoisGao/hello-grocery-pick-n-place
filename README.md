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


## 5. Implementation and Running Instructions

This project is implemented as a ROS 2-based perception-to-manipulation pipeline for grocery pick-and-place using a UR3e robot arm, Intel RealSense RGB-D camera, YOLOv11 object detection, MoveIt motion planning, and a custom SO-101 gripper controller.

The current implementation follows this pipeline:

Camera image/depth stream → YOLO object detection → object 3D pose estimation → TF transformation to `base_link` → pick pose planning → MoveIt trajectory execution → gripper open/close control

Before running any ROS 2 node, source the ROS 2 Kilted environment and the workspace:

```bash
source /opt/ros/kilted/setup.bash
source ~/ros2_ws/install/setup.bash

## Terminal 1: Start the UR3e Driver

Launch the Universal Robots driver and connect to the physical UR3e robot:

source /opt/ros/kilted/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=10.3.4.11 \
  launch_rviz:=false

## Terminal 2: Start MoveIt

Launch the MoveIt configuration for the UR3e arm:

source /opt/ros/kilted/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e \
  robot_ip:=10.3.4.11

Optional motion test:

ros2 run ur3e_demo move_client_with_env

## Terminal 3: Start the RealSense Camera

Launch the RealSense RGB-D camera with aligned depth enabled:

source /opt/ros/kilted/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30

## Terminal 4: Run YOLOv11 Object Detection

Run YOLOv11 on the RealSense color image topic:

source /opt/ros/kilted/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch yolo_bringup yolo.launch.py \
  model:=yolo11n.pt \
  input_image_topic:=/camera/camera/color/image_raw \
  image_reliability:=2 \
  device:=cpu \
  threshold:=0.4 \
  imgsz_height:=320 \
  imgsz_width:=320 \
  use_debug:=True

To visualize YOLO detection results, open RViz2 and add the debug image topic:

rviz2

In RViz2:

Add → By topic → /yolo/dbg_image → Image
Fixed Frame = base_link

Alternatively, use rqt_image_view:

ros2 run rqt_image_view rqt_image_view /yolo/dbg_image

## Terminal 5: Publish the Camera Mount TF

Launch the project bringup file to publish the static transform between the robot tool frame and the camera frame:

source /opt/ros/kilted/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch grocery_robot_bringup grocery_robot_bringup.launch.py

This should publish the transform:

tool0 → camera_link

## Terminal 6: Run the Object Pose Estimator

Estimate the 3D position of the target object from YOLO detections and aligned depth data:

source /opt/ros/kilted/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run grocery_perception object_pose_estimator --ros-args \
  -p target_class:=orange \
  -p min_confidence:=0.35 \
  -p min_stable_count:=5 \
  -p buffer_size:=10 \
  -p max_position_std_m:=0.025

## Terminal 7: Transform Object Pose to Robot Base Frame

Transform the detected object pose from the camera frame into the UR3e base_link frame:

source /opt/ros/kilted/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run grocery_perception object_pose_transformer --ros-args \
  -p target_frame:=base_link

## Terminal 8: Run the Pick Planner

Generate pre-grasp, grasp, and lift poses based on the transformed object position:

source /opt/ros/kilted/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run grocery_perception pick_planner --ros-args \
  -p pre_grasp_z_offset:=0.20 \
  -p grasp_z_offset:=0.08 \
  -p lift_z_offset:=0.28

## Terminal 9: Execute the Pick Motion

Run the pick executor to send the planned poses to MoveIt and execute the robot trajectory:

source /opt/ros/kilted/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run grocery_perception pick_executor --ros-args \
  -p move_group_action:=/move_action \
  -p planning_group:=ur_manipulator \
  -p eef_link:=tool0 \
  -p plan_only:=false \
  -p velocity_scaling:=0.02 \
  -p acceleration_scaling:=0.02 \
  -p position_tolerance:=0.03 \
  -p orientation_tolerance:=0.50

For safer testing, set:

-p plan_only:=true

This allows MoveIt to plan the motion without executing it on the real robot.

## Terminal 10: Run the SO-101 Gripper Node

The SO-101 gripper node runs inside the lerobot conda environment:

source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot

Then start the gripper node:

ros2 run so101_gripper_control gripper_node --ros-args \
  -p port:=/dev/ttyACM0 \
  -p servo_id:=6 \
  -p open_position:=3039 \
  -p close_position:=2077 \
  -p speed:=300

Expected output:

[INFO] [so101_gripper_node]: SO-101 gripper node started on /dev/ttyACM0, servo ID 6

To manually test the gripper, publish open and close commands from another terminal:

source /opt/ros/kilted/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic pub /gripper_cmd std_msgs/msg/String "data: 'open'" --once
ros2 topic pub /gripper_cmd std_msgs/msg/String "data: 'close'" --once

## Gripper Setup

The project supports two gripper modes: a real SO-101 gripper mode and a mock gripper mode.

### Real SO-101 Gripper

The real gripper requires the `lerobot` conda environment, the SO-101 servo SDK, serial access to the controller board, and the physical gripper connected through `/dev/ttyACM0`.

Activate the `lerobot` environment:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
