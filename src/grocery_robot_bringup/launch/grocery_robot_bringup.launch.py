from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    world_to_base_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="world_to_base_link_tf",
    arguments=[
        "0", "0", "0",      # x y z
        "0", "0", "0",      # roll pitch yaw
        "world",
        "base_link"
    ],
)

    tool0_to_camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tool0_to_camera_link_tf',
        arguments=[
            '0.00', '-0.10', '0.03',
            '0', '0', '0',
            'tool0',
            'camera_link'
        ],
        output='screen'
    )

    camera_link_to_color_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_color_optical_tf',
        arguments=[
            '0', '0', '0',
            '-1.5708', '0', '-1.5708',
            'camera_link',
            'camera_color_optical_frame'
        ],
        output='screen'
    )

    return LaunchDescription([
        world_to_base_tf,
        tool0_to_camera_link_tf,
        camera_link_to_color_optical_tf,
    ])