#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32

import scservo_sdk as scs


# Feetech / SCServo control table addresses.
# These are the common addresses used by the Feetech SCS/STS SDK examples.
ADDR_GOAL_POSITION = 42
ADDR_GOAL_TIME = 44
ADDR_GOAL_SPEED = 46
ADDR_PRESENT_POSITION = 56


class SO101GripperNode(Node):
    def __init__(self):
        super().__init__("so101_gripper_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("servo_id", 6)

        # -------------------------------------------------------
        # TUNE THESE TWO VALUES FOR YOUR PHYSICAL GRIPPER
        #
        # open_position:
        #   Raw servo position where the gripper is safely open.
        #
        # close_position:
        #   Raw servo position where the gripper is closed enough
        #   to hold an object, but NOT over-tightened.
        #
        # Test small changes first:
        #   ros2 topic pub /gripper_cmd std_msgs/msg/String "data: 'pos 1800'" --once
        #   ros2 topic pub /gripper_cmd std_msgs/msg/String "data: 'pos 1900'" --once
        #   ros2 topic pub /gripper_cmd std_msgs/msg/String "data: 'pos 2000'" --once
        # -------------------------------------------------------
        self.declare_parameter("open_position", 3039)
        self.declare_parameter("close_position", 2077)
        self.declare_parameter("speed", 300)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.servo_id = int(self.get_parameter("servo_id").value)

        self.open_position = int(self.get_parameter("open_position").value)
        self.close_position = int(self.get_parameter("close_position").value)
        self.speed = int(self.get_parameter("speed").value)

        self.port_handler = scs.PortHandler(self.port)
        self.packet_handler = scs.PacketHandler(0)

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open serial port: {self.port}")

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"Failed to set baudrate: {self.baudrate}")

        self.cmd_sub = self.create_subscription(
            String,
            "/gripper_cmd",
            self.command_callback,
            10,
        )

        self.position_pub = self.create_publisher(
            Int32,
            "/gripper/raw_position",
            10,
        )

        self.timer = self.create_timer(0.2, self.publish_position)

        self.get_logger().info(
            f"SO-101 gripper node started on {self.port}, servo ID {self.servo_id}"
        )

    def command_callback(self, msg):
        command = msg.data.strip().lower()

        if command == "open":
            self.get_logger().info(f"Opening gripper to {self.open_position}")
            self.move_to(self.open_position)

        elif command == "close":
            self.get_logger().info(f"Closing gripper to {self.close_position}")
            self.move_to(self.close_position)

        elif command.startswith("pos"):
            try:
                _, value = command.split()
                target_position = int(value)
                self.get_logger().info(f"Moving gripper to raw position {target_position}")
                self.move_to(target_position)

            except ValueError:
                self.get_logger().error("Invalid command. Use: pos <number>")

        else:
            self.get_logger().warn("Unknown command. Use: open, close, or pos <number>")

    def move_to(self, position):
        """
        Move SO-101 gripper servo to a raw Feetech servo position.

        Your calibrated values:
        open  = 3039
        close = 2077

        Note:
        close_position < open_position, so the gripper closes as the raw
        servo position decreases.
        """

        open_limit = self.open_position      # 3039
        close_limit = self.close_position    # 2077

        # Safety clamp:
        # Because close_limit < open_limit, valid range is [2077, 3039].
        min_limit = min(open_limit, close_limit)
        max_limit = max(open_limit, close_limit)

        safe_position = max(min_limit, min(max_limit, int(position)))

        self.get_logger().info(
            f"Commanding gripper to {safe_position} "
            f"(open={open_limit}, close={close_limit})"
        )

        # Set speed first.
        # Lower speed is safer during tuning.
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler,
            self.servo_id,
            ADDR_GOAL_SPEED,
            int(self.speed),
        )

        self.get_logger().info(f"Speed write result={result}, error={error}")

        if result != scs.COMM_SUCCESS:
            self.get_logger().warn(f"Speed write failed: {result}")

        if error != 0:
            self.get_logger().warn(f"Servo returned error after speed write: {error}")

        # Then command target position.
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler,
            self.servo_id,
            ADDR_GOAL_POSITION,
            safe_position,
        )

        self.get_logger().info(f"Position write result={result}, error={error}")

        if result != scs.COMM_SUCCESS:
            self.get_logger().warn(f"Position write failed: {result}")

        if error != 0:
            self.get_logger().warn(f"Servo returned error after position write: {error}")

    def publish_position(self):
        try:
            position, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler,
                self.servo_id,
                ADDR_PRESENT_POSITION,
            )

            if result == scs.COMM_SUCCESS and error == 0:
                msg = Int32()
                msg.data = int(position)
                self.position_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Could not read servo position: {e}")

    def destroy_node(self):
        try:
            self.port_handler.closePort()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SO101GripperNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()