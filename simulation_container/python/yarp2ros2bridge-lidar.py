import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import yarp
import math
import numpy as np

last_ranges = None

def is_duplicate_scan(current_ranges):
    global last_ranges
    if last_ranges is None:
        last_ranges = np.array(current_ranges)
        return False  # First message, no duplicates yet

    current_ranges_np = np.array(current_ranges)
    if np.array_equal(last_ranges, current_ranges_np):
        return True  # Duplicate detected

    last_ranges = current_ranges_np
    return False

class YARPLidarToROS2(Node):
    def __init__(self):
        super().__init__('Ergocub_lidar')

        # ROS 2 publisher
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        # Initialize YARP
        yarp.Network.init()

        # Create a YARP BufferedPort to read LIDAR data
        self.lidar_port = yarp.BufferedPortBottle()
        if not self.lidar_port.open("/lidar_reader"):
            self.get_logger().error("Failed to open YARP port for LIDAR")

        # Connect to the LIDAR output (ensure correct YARP port)
        if not yarp.Network.connect("/ergocubSim/laser:o", "/lidar_reader"):
            self.get_logger().error("Failed to connect to LIDAR YARP port")

        self.timer = self.create_timer(0.01, self.read_lidar_data)  # Read at 100 Hz



    def read_lidar_data(self):
        """ Reads LIDAR data from YARP and publishes it to ROS 2 """
        lidar_bottle = self.lidar_port.read()
        if lidar_bottle is None:
            self.get_logger().warning("No LIDAR data received")
            return

        # Parse LIDAR components
        start_angle = lidar_bottle.get(0).asFloat64()
        end_angle = lidar_bottle.get(1).asFloat64()
        min_range = lidar_bottle.get(2).asFloat64()
        max_range = lidar_bottle.get(3).asFloat64()

        lidar_measurements = [
            lidar_bottle.get(4).asList().get(i).asFloat64()
            for i in range(lidar_bottle.get(4).asList().size())
        ]

        # Publish to ROS 2
        laser_msg = LaserScan()
        laser_msg.header.stamp = self.get_clock().now().to_msg()
        laser_msg.header.frame_id = "lidar_frame"
        laser_msg.angle_min = start_angle
        laser_msg.angle_max = math.radians(end_angle)
        laser_msg.angle_increment = 0.008726646192371845
        laser_msg.time_increment = 0.01 / len(lidar_measurements)
        laser_msg.scan_time = 0.01  # Set to 10 ms (100 Hz)
        laser_msg.range_min = min_range
        laser_msg.range_max = max_range
        laser_msg.ranges = lidar_measurements

        if not is_duplicate_scan(laser_msg.ranges):
            print("⚠️ Duplicate LiDAR scan detected! Skipping message...")
            return

        self.publisher.publish(laser_msg)
        self.get_logger().info(f"Published LIDAR data with {len(lidar_measurements)} points.")



    def shutdown(self):
        """ Closes YARP connections on shutdown """
        self.get_logger().info("Closing LIDAR YARP port...")
        self.lidar_port.close()
        yarp.Network.fini()

def main(args=None):
    rclpy.init(args=args)
    node = YARPLidarToROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LIDAR bridge...")
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

