from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ergocub_slam_bridge',
            executable='yarp2ros2bridge-lidar',
            name='yarp_lidar_bridge',
            output='screen'
        ),
        Node(
            package='ergocub_slam_bridge',
            executable='yarp2ros2bridge-imu-head',
            name='yarp_imu_bridge',
            output='screen'
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
    ])
