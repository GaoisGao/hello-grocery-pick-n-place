from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command


def generate_launch_description():

    # Find our package
    ur3e_gazebo_pkg = FindPackageShare('ur3e_gazebo')

    # Generate the URDF from our wrapper xacro (which lives in ur3e_gazebo, not the upstream package).
    # ParameterValue(..., value_type=str) prevents ROS 2 from trying to parse the URDF XML as YAML.
    robot_description = ParameterValue(
        Command([
            'ros2 run xacro xacro ',
            PathJoinSubstitution([ur3e_gazebo_pkg, 'urdf', 'ur3e_gz.urdf.xacro']),
        ]),
        value_type=str,
    )

    # Start Gazebo with our world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': PathJoinSubstitution([
            FindPackageShare('ur3e_gazebo'), 'worlds', 'grocery_world.sdf'
        ])}.items(),
    )

    # Bridge 1: /clock only — critical for controller_manager sim time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    # Bridge 2: all camera topics — arm and overhead RGB + depth streams
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Arm RGB camera
            '/arm_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/arm_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Arm depth camera
            '/arm_depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/arm_depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/arm_depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/arm_depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Overhead RGB camera
            '/overhead_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Overhead depth camera
            '/overhead_depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/overhead_depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/overhead_depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/overhead_depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
    )

    # Publish the robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Spawn the UR3e into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ur3e',
            '-topic', 'robot_description',
        ],
    )

    # Spawn joint_state_broadcaster after the robot is in Gazebo
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Spawn joint_trajectory_controller after joint_state_broadcaster is active
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
    )

    # Send the home pose command after joint_trajectory_controller is active.
    # Gripper fingers are now part of joint_trajectory_controller — no separate
    # gripper spawner needed.
    # TO SWITCH TO ROBOTIQ 2F-85: restore a gripper_controller_spawner node here
    # and chain it between joint_trajectory_controller and home_pose.
    home_pose = Node(
        package='ur3e_gazebo',
        executable='home_pose',
    )

    # Chain: spawn_robot -> joint_state_broadcaster -> joint_trajectory_controller -> home_pose
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    load_home_pose = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[home_pose],
        )
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        camera_bridge,
        robot_state_publisher,
        spawn_robot,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
        load_home_pose,
    ])