from launch import LaunchDescription
from launch_ros.actions.node import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

urdf_path = os.path.join(get_package_share_directory('arm6dof'), 'urdf', 'arm6dof.urdf')
robot_description = open(urdf_path).read()


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['stty', '-F', '/dev/ttyUSB0', '2000000'], output='screen'),
        Node(
            namespace="arm6dof",
            executable="armnode",
            package="arm6dof"

        ),
        # Node(
        #     executable="robot_state_publisher",
        #     package="robot_state_publisher",
        #     parameters=[{'robot_description': robot_description}]
        # ),
        # Node(
        #     executable='joint_state_publisher',
        #     package="joint_state_publisher"
        # )

    ])
