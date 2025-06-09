#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import yarp


class YARPOdometryBridge(Node):
    def __init__(self):
        super().__init__('yarp_odom_bridge')

        # ROS 2 publisher
        self.odom_pub = self.create_publisher(Odometry, '/walking/odom', 10)

        # YARP port
        self.port = yarp.BufferedPortBottle()
        self.port.open('/yarp2ros2/odom:i')

        # Connect to YARP odometry output
        yarp.Network.connect('/walking/odom:o', '/yarp2ros2/odom:i')

        # Timer to read at 100 Hz
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        bottle = self.port.read(False)
        if bottle is None or bottle.size() < 7:
            return

        odom_msg = Odometry()
        now = self.get_clock().now().to_msg()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Parse position and orientation
        odom_msg.pose.pose.position.x = bottle.get(0).asFloat64()
        odom_msg.pose.pose.position.y = bottle.get(1).asFloat64()
        odom_msg.pose.pose.position.z = bottle.get(2).asFloat64()

        odom_msg.pose.pose.orientation.x = bottle.get(3).asFloat64()
        odom_msg.pose.pose.orientation.y = bottle.get(4).asFloat64()
        odom_msg.pose.pose.orientation.z = bottle.get(5).asFloat64()
        odom_msg.pose.pose.orientation.w = bottle.get(6).asFloat64()

        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    yarp.Network.init()
    node = YARPOdometryBridge()
    rclpy.spin(node)
    node.destroy_node()
    yarp.Network.fini()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
