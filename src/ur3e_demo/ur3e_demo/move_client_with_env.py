#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    JointConstraint,
    Constraints,
    MoveItErrorCodes
)

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject
from std_msgs.msg import String

# ================================
# 🔧 CONFIGURATION
# ================================

# Replace with your desired joint configs
HOME = [0.00,
    -1.57,
    0.00,
    -1.57,
    0.00,
    0.00]

OBSERVATION = [-1.68,
    -1.97,
    0.01,
    -1.07,
    1.60,
    0.00]

PREGRASP = [-1.45,
    -2.51,
    0.12,
    -0.76,
    1.53,
    0.12]

GRASP = [-0.96,
    -3.00,
    0.10,
    -0.76,
    1.53,
    0.10]

LOWER = [0.62,
    0.41,
    0.22,
    -3.16,
    -0.37,
    -0.96]

MIDDLE = [1.10,
    -0.11,
    0.18,
    -3.52,
    -0.18,
    -0.59]

MIDDLE_PLACE = [0.96,
    -0.13,
    0.35,
    -3.42,
    -0.38,
    -0.56]

UPPER = [1.04,
    -0.56,
    0.12,
    -3.69,
    -0.15,
    0.05]


JOINT_NAMES = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]



# ================================
# 🤖 ACTION CLIENT NODE
# ================================

class UR3eMoveClient(Node):

    def __init__(self):
        super().__init__("ur3e_move_client")

        self.client = ActionClient(self, MoveGroup, "/move_action")

        self.get_logger().info("Waiting for MoveIt action server...")
        self.client.wait_for_server()
        self.get_logger().info("Connected to MoveIt.\n")

        self.scene_pub = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )
        self.gripper_pub = self.create_publisher(
            String,
            "/gripper_cmd",
            10
        
        )

    def move_to_joint_state(self, name, joint_positions):
        self.get_logger().info(f"Planning to: {name}")

        request = MotionPlanRequest()

        # Basic setup
        request.group_name = "ur_manipulator"
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3

        # Workspace bounds
        request.workspace_parameters.header.frame_id = "base_link"
        request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.0)

        # Joint constraints
        constraints = Constraints()

        for joint_name, pos in zip(JOINT_NAMES, joint_positions):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request.goal_constraints.append(constraints)

        # Wrap into goal
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        goal.planning_options.planning_scene_diff.is_diff = True

        # Send goal
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected\n")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"Reached {name}\n")
            return True
        else:
            self.get_logger().error(
                f"Failed {name}, error code: {result.error_code.val}\n"
            )
            return False

    def command_gripper(self, command):
        """
        Send command to SO-101 gripper node.

        Valid commands:
        "open"
        "close"
        """
        msg = String()
        msg.data = command

        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Sent gripper command: {command}")

    def add_banana(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        banana = CollisionObject()
        banana.id = "banana"
        banana.header.frame_id = "base_link"

        # Shape (cylinder or box approximation)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [0.10, 0.08]  
        # height, radius

        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.0
        pose.position.z = 0.3
        pose.orientation.w = 1.0

        banana.primitives.append(primitive)
        banana.primitive_poses.append(pose)
        banana.operation = CollisionObject.ADD

        scene.world.collision_objects.append(banana)

        self.scene_pub.publish(scene)

        self.get_logger().info("Banana added to planning scene")

    def add_shelf(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        shelf = CollisionObject()
        shelf.id = "rectangular_shelf"
        shelf.header.frame_id = "base_link"

        # Shelf dimensions in meters:
        # 580 mm x 280 mm x 250 mm
        # For SolidPrimitive.BOX, dimensions order is:
        # [x_length, y_width, z_height]
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            0.28,   # x dimension:
            0.25,   # y dimension:
            0.58    # z dimension:
        ]

        pose = Pose()

        # ==============================
        # TUNE SHELF POSITION HERE
        # ==============================
        pose.position.x = 0.40   # shelf center x position relative to base_link
        pose.position.y = 0.60   # shelf center y position, about 60 cm from arm base
        pose.position.z = 0.10   # shelf center z position, about 30 cm above base plane
        # ==============================

        pose.orientation.w = 1.0

        shelf.primitives.append(primitive)
        shelf.primitive_poses.append(pose)
        shelf.operation = CollisionObject.ADD

        scene.world.collision_objects.append(shelf)

        self.scene_pub.publish(scene)

        self.get_logger().info(
            "Rectangular shelf added to planning scene: "
            "size = 0.58 x 0.28 x 0.25 m, "
            "pose = (x=0.0, y=0.60, z=0.30)"
        )
    
    def attach_camera_gripper_collision(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        attached_object = AttachedCollisionObject()
        attached_object.link_name = "tool0"   # attach to end-effector frame

        collision_object = CollisionObject()
        collision_object.id = "mounted_camera_gripper"
        collision_object.header.frame_id = "tool0"

        # Camera + gripper approximation
        # Dimensions are in meters:
        # x = 13 cm, y = 14 cm, z = 14 cm
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            0.13,   # x length
            0.14,   # y width
            0.08    # z height
        ]

        pose = Pose()

        # ==============================
        # TUNE MOUNTED OBJECT POSITION HERE
        # Relative to tool0 frame
        # ==============================
        pose.position.x = 0.00
        pose.position.y = 0.00
        pose.position.z = 0.07
        # ==============================

        pose.orientation.w = 1.0

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        attached_object.object = collision_object

        # Allow the attached object to touch nearby end-effector links.
        # You may need to adjust these names based on your TF/URDF.
        attached_object.touch_links = [
            "tool0",
            "flange",
            "wrist_3_link"
        ]

        scene.robot_state.attached_collision_objects.append(attached_object)

        self.scene_pub.publish(scene)

        self.get_logger().info(
            "Attached mounted camera/gripper collision object to tool0: "
            "size = 0.13 x 0.14 x 0.14 m"
        )

# ================================
# 🚀 MAIN EXECUTION
# ================================

def main():
    rclpy.init()

    node = UR3eMoveClient()

    node.add_shelf()
    time.sleep(1)

    node.attach_camera_gripper_collision()
    time.sleep(1)

    success = node.move_to_joint_state("OBSERVATION", OBSERVATION)
    if success:
        node.command_gripper("open")
    time.sleep(2)

    node.move_to_joint_state("PREGRASP", PREGRASP)
    time.sleep(1)

    success = node.move_to_joint_state("GRASP", GRASP)
    if success:
       node.command_gripper("close")
    time.sleep(3)

    node.move_to_joint_state("HOME", HOME)
    time.sleep(1)

    success = node.move_to_joint_state("MIDDLE", MIDDLE)
    if success:
        node.command_gripper("open")
    time.sleep(1)

    # success = node.move_to_joint_state("MIDDLE_PLACE", MIDDLE_PLACE)
    # if success:
    #     node.command_gripper("open")
    # time.sleep(2)

    # node.move_to_joint_state("MIDDLE", MIDDLE)
    # time.sleep(1)

    # node.move_to_joint_state("HOME", HOME)
    # time.sleep(1)
    # node.move_to_joint_state("HOME", HOME)
    # time.sleep(1)

    # node.move_to_joint_state("OBSERVATION", OBSERVATION)
    # time.sleep(1)

    # node.move_to_joint_state("HOME", HOME)
    # time.sleep(1)

    # node.move_to_joint_state("LOWER", LOWER)
    # time.sleep(1)

    # node.move_to_joint_state("HOME", HOME)
    # time.sleep(1)







    #node.move_to_joint_state("HOME", HOME)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
