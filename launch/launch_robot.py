from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = True

    # Configuration for RPLidar
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') #for A1/A2 is 115200
    frame_id = LaunchConfiguration('frame_id', default='laser')
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

        # Argument for slam_toolbox

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(get_package_share_directory("slam_toolbox"),
                                       'config', 'mapper_params_online_sync.yaml'),
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy',
            #parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='pandabot_drive',
            executable='pandabot_teleop',
            name='pandabot_teleop',
            #parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='pandabot_drive',
            executable='pandabot_drive',
            name='pandabot_drive',
            #parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='pandabot_drive',
            executable='pandabot_odometry',
            name='pandabot_odometry',
            #parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='rplidar_ros2',
            executable='rplidar_scan_publisher',
            name='rplidar_scan_publisher',
            parameters=[{'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate,
                         #'use_sim_time': use_sim_time
						 }],
            output='screen'),
		Node(
			package='tf2_ros',
			executable='static_transform_publisher',
			output='screen',
			arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
		)
    ])
