from setuptools import setup

package_name = 'ergocub_slam_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ergocub_slam_bridge']),
        ('share/ergocub_slam_bridge', ['package.xml']),
        ('share/ergocub_slam_bridge/launch', ['launch/slam_with_yarp.launch.py']),
        ('share/ergocub_slam_bridge/config', ['config/slam_toolbox_params.yaml']),
        ('share/ergocub_slam_bridge/config', ['config/ekf.yaml']),
        ('share/ergocub_slam_bridge/config', ['config/main.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='your.email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yarp2ros2bridge-lidar = ergocub_slam_bridge.yarp2ros2bridge_lidar:main',
            'yarp2ros2bridge-imu-head = ergocub_slam_bridge.yarp2ros2bridge_imu_head:main',
            'yarp2ros2bridge-imu-waist = ergocub_slam_bridge.yarp2ros2bridge_imu_waist:main',
            'dynamic_tf_broadcaster = ergocub_slam_bridge.dynamic_tf_broadcaster:main',
            'yarp2ros2bridge_odom = ergocub_slam_bridge.yarp2ros2bridge_odom:main',
        ],
    },
)
