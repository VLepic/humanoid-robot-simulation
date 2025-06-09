import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import yarp
import time
import math
from tf_transformations import quaternion_from_euler


class YarpToRos2TFBridge(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)

        # YARP initialization
        yarp.Network.init()
        self.port = yarp.BufferedPortBottle()
        self.port.open("/yarp2ros2_tf_bridge:i")

        if not yarp.Network.connect("/walking/tf:o", "/yarp2ros2_tf_bridge:i"):
            self.get_logger().error("Cannot connect to /walking/tf:o")
            exit(1)

        self.static_transforms = [
            self.create_transform("head_imu_0", "head_imu_frame", 0.0, 0.0, 0.0, 2*math.pi, 0, 0),
            self.create_transform("waist_imu_0", "waist_imu_frame", 0.0, 0.0, 0.0, 0, 2*math.pi, 0),
            self.create_transform("realsense", "realsense_frame", 0.0, 0.0, 0.0, 0, 2*math.pi, 0),
            self.create_transform("head_laser_frame", "lidar_frame", 0.0, 0.0, 0.0, 0, 0, 0),
        ]

        self.timer = self.create_timer(0.01, self.timer_callback)

    def create_transform(self, parent, child, x, y, z, roll, pitch, yaw):
        q = quaternion_from_euler(roll, pitch, yaw)
        t = TransformStamped()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # Broadcast static transforms every cycle
        for t in self.static_transforms:
            t.header.stamp = now
            self.tf_broadcaster.sendTransform(t)

        # Read and broadcast dynamic YARP transforms
        bottle = self.port.read(False)
        if bottle is None:
            return

        for i in range(bottle.size()):
            tf_bottle = bottle.get(i).asList()

            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = tf_bottle.get(0).asString()
            t.child_frame_id = tf_bottle.get(1).asString()
            t.transform.translation.x = tf_bottle.get(2).asFloat64()
            t.transform.translation.y = tf_bottle.get(3).asFloat64()
            t.transform.translation.z = tf_bottle.get(4).asFloat64()
            t.transform.rotation.x = tf_bottle.get(5).asFloat64()
            t.transform.rotation.y = tf_bottle.get(6).asFloat64()
            t.transform.rotation.z = tf_bottle.get(7).asFloat64()
            t.transform.rotation.w = tf_bottle.get(8).asFloat64()

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = YarpToRos2TFBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    yarp.Network.fini()


if __name__ == '__main__':
    main()


