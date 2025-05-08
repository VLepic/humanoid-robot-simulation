from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

pkg_share = get_package_share_directory('ergocub_slam_bridge')



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
            package='ergocub_slam_bridge',
            executable='yarp2ros2bridge-imu-waist',
            name='yarp_imu_bridge_waist',
            output='screen'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('ergocub_slam_bridge'),
                'config',
                'slam_toolbox_params.yaml'
            ])]
        ),
        Node(
            package='ergocub_slam_bridge',
            executable='dynamic_tf_broadcaster',
            name='dynamic_tf_broadcaster',
            output='screen'
        ),
        Node(
            package='ergocub_slam_bridge',
            executable='yarp2ros2bridge_odom',
            name='yarp2ros2bridge_odom',
            output='screen'
        ),
        Node(
            package='ergocub_slam_bridge',
            executable='yarp2ros2bridge_odom',
            name='yarp2ros2bridge_odom',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config', 'main.rviz')],
            output='screen'
        )


    ])
