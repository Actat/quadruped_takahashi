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
            'publish_frequency': 1000.0,
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

    imu_node = LifecycleNode(
        package='rt_usb_9axisimu_driver',
        executable='rt_usb_9axisimu_driver',
        name='rt_usb_9axisimu',
        parameters=[{
            'port': '/dev/ttyACM0'
        }]
    )
    imu_inactive_to_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=imu_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            imu_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )]
        )
    )
    imu_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(imu_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_complementary_filter',
        parameters=[{
            'use_mag': True,
            'publish_tf': False,
        }]
    )

    kondo_b3m_ros2_node = Node(
        package='kondo_b3m_ros2',
        executable='kondo_b3m',
        remappings=[('~/joint_states', '/joint_states')],
        parameters=[{
            'publish_frequency': 50,
            'motor_list': [
                "{'id': 0, 'model': 'B3M-SC-1170-A', 'name': 'lf0', 'direction': False}",
                "{'id': 1, 'model': 'B3M-SC-1170-A', 'name': 'lf1', 'direction': False}",
                "{'id': 2, 'model': 'B3M-SC-1170-A', 'name': 'lf2', 'direction': False}",
                "{'id': 3, 'model': 'B3M-SC-1170-A', 'name': 'rf0', 'direction': False}",
                "{'id': 4, 'model': 'B3M-SC-1170-A', 'name': 'rf1'}",
                "{'id': 5, 'model': 'B3M-SC-1170-A', 'name': 'rf2'}",
                "{'id': 6, 'model': 'B3M-SC-1170-A', 'name': 'lh0'}",
                "{'id': 7, 'model': 'B3M-SC-1170-A', 'name': 'lh1', 'direction': False}",
                "{'id': 8, 'model': 'B3M-SC-1170-A', 'name': 'lh2', 'direction': False}",
                "{'id': 9, 'model': 'B3M-SC-1170-A', 'name': 'rh0'}",
                "{'id': 10, 'model': 'B3M-SC-1170-A', 'name': 'rh1'}",
                "{'id': 11, 'model': 'B3M-SC-1170-A', 'name': 'rh2'}"
            ]}],
    )

    quadruped_takahashi_odometry_node = Node(
        package='quadruped_takahashi',
        executable='quadruped_takahashi_odometry_node',
        remappings=[('~/tf', 'tf')],
    )

    quadruped_takahashi_control_node = Node(
        package='quadruped_takahashi',
        executable='quadruped_takahashi_control_node',
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(static_transform_publisher_node)
    ld.add_action(imu_inactive_to_active)
    ld.add_action(imu_node)
    ld.add_action(imu_configure)
    ld.add_action(imu_complementary_filter_node)
    ld.add_action(rviz_node)
    ld.add_action(kondo_b3m_ros2_node)
    ld.add_action(quadruped_takahashi_odometry_node)
    ld.add_action(quadruped_takahashi_control_node)

    return ld
