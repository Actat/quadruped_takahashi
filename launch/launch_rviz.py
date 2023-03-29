import os
import launch
from launch import LaunchIntrospector

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    rviz_file_name = 'rviz_config.rviz'
    rviz_path = os.path.join(
        get_package_share_directory('quadruped_takahashi'),
        'rviz',
        rviz_file_name
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path]
    )

    ld = LaunchDescription()
    ld.add_action(rviz_node)

    return ld
