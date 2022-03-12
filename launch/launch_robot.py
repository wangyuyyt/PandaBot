from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Configuration for RPLidar
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') #for A1/A2 is 115200
    frame_id = LaunchConfiguration('frame_id', default='base_scan')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')

    return LaunchDescription([
        # Argument for RPLidar
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        # Launch state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
               [ThisLaunchFileDir(), '/pandabot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy',
        ),
        Node(
            package='pandabot_drive',
            executable='pandabot_teleop',
            name='pandabot_teleop',
        ),
        Node(
            package='pandabot_drive',
            executable='pandabot_drive',
            name='pandabot_drive',
        ),
        Node(
            package='pandabot_drive',
            executable='pandabot_odometry',
            name='pandabot_odometry',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_composition',
            parameters=[{'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate,
                         'use_sim_time': use_sim_time
						 }],
            output='screen'),
    ])
