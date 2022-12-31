import os
import launch
from launch import LaunchIntrospector

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.substitutions import LaunchConfiguration

import lifecycle_msgs.msg


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'quadruped_takahashi.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('quadruped_takahashi'),
        'urdf',
        urdf_file_name
    )
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    rviz_file_name = 'rviz_config.rviz'
    rviz_path = os.path.join(
        get_package_share_directory('quadruped_takahashi'),
        'rviz',
        rviz_file_name
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
        }],
        arguments=[urdf_path],
    )
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path]
    )

    kondo_b3m_ros2_node = Node(
        package='kondo_b3m_ros2',
        executable='kondo_b3m',
        remappings=[('b3m_joint_state', 'joint_states')],
        parameters=[{'motor_list': [
            "{'id': 0, 'name': 'lf0', 'direction': False}",
            "{'id': 1, 'name': 'lf1', 'direction': False}",
            "{'id': 2, 'name': 'lf2', 'direction': False}",
            "{'id': 3, 'name': 'rf0', 'direction': False}",
            "{'id': 4, 'name': 'rf1'}",
            "{'id': 5, 'name': 'rf2'}",
            "{'id': 6, 'name': 'lh0'}",
            "{'id': 7, 'name': 'lh1', 'direction': False}",
            "{'id': 8, 'name': 'lh2', 'direction': False}",
            "{'id': 9, 'name': 'rh0'}",
            "{'id': 10, 'name': 'rh1'}",
            "{'id': 11, 'name': 'rh2'}"
        ]}],
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(static_transform_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(kondo_b3m_ros2_node)

    return ld
