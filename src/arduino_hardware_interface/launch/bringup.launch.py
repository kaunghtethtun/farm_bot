from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = LaunchConfiguration('urdf_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(
                '/home/khh/ttu_ws/install/arduino_hardware_interface/share/arduino_hardware_interface/description',
                'robot.urdf.xacro'
            ),
            description='Full path to robot urdf file'
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': Command(['xacro ', urdf_file])},
                os.path.join(
                    '/home/khh/ttu_ws/install/arduino_hardware_interface/share/arduino_hardware_interface/config',
                    'controller_manager.yaml'
                )
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager-timeout', '50'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--param-file',
                os.path.join(
                    '/home/khh/ttu_ws/install/arduino_hardware_interface/share/arduino_hardware_interface/config',
                    'diff_drive_controller.yaml'
                ),
                '--controller-manager-timeout', '50'
            ],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}],
            output='screen',
        ),
    ])
