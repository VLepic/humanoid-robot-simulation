import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import yarp
import numpy as np


class IMUBridge(Node):
    def __init__(self):
        super().__init__('Ergocub_imu_head')

        # Initialize YARP
        yarp.Network.init()

        # Open YARP port
        self.imu_port = yarp.BufferedPortBottle()
        if not self.imu_port.open("/imu_reader"):
            self.get_logger().error("Failed to open YARP port")

        # Connect to the IMU output port
        yarp.Network.connect("/ergocubSim/head/inertials/measures:o", "/imu_reader")

        # ROS2 IMU publisher
        self.imu_publisher = self.create_publisher(Imu, 'imu_head_data', 10)

        # Store last received IMU data to filter duplicates
        self.last_acc = None
        self.last_gyro = None
        self.last_mag = None

        # Timer to fetch data at a fixed rate
        self.timer = self.create_timer(0.01, self.read_imu)  # 100 Hz

    def read_imu(self):
        imu_bottle = self.imu_port.read()  # Read data from YARP
        if imu_bottle is None:
            self.get_logger().warn("No IMU data received")
            return

        # Extract IMU components
        gyro, gyro_timestamp = self.extract(imu_bottle.get(0))
        acc, acc_timestamp = self.extract(imu_bottle.get(1))
        mag, mag_timestamp = self.extract(imu_bottle.get(2))
        orient, orient_timestamp = self.extract(imu_bottle.get(3))


        # Check for duplicate data
        if np.array_equal(acc, self.last_acc) and np.array_equal(gyro, self.last_gyro) and np.array_equal(mag,self.last_mag):
            print("⚠️ Duplicate LiDAR scan detected! Skipping message...")
            return  # Skip publishing duplicate data

        self.last_acc, self.last_gyro, self.last_mag = acc, gyro, mag

        # Create IMU ROS2 message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_frame"

        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = acc
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = gyro
        imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z = orient



        # Publish to ROS2
        self.imu_publisher.publish(imu_msg)
        self.get_logger().info(f"Published IMU data: Acc: {acc}, Gyro: {gyro}")

    def extract(self, acc_list):
        if acc_list.isList():
            acc_list = acc_list.asList()
            inner_list = acc_list.get(0)
            if inner_list.isList():
                timestamp = inner_list.asList().get(1).asFloat64()
                values = inner_list.asList().get(0).asList()
                extracted_values = [values.get(i).asFloat64() for i in range(values.size())]
                return extracted_values, timestamp
        return None, None


def main(args=None):
    rclpy.init(args=args)
    imu_bridge = IMUBridge()
    rclpy.spin(imu_bridge)
    imu_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()