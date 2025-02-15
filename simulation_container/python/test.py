import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np



class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'lidar',  # Topic name
            self.lidar_callback,
            10  # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning

        # Initialize Matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.ax.set_xlim(-5, 5)  # Adjust based on expected range
        self.ax.set_ylim(-5, 5)
        self.ax.set_title("Live LiDAR Scan")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.scatter = self.ax.scatter([], [])  # Empty scatter plot
        plt.ion()  # Interactive mode

    def lidar_callback(self, msg: LaserScan):
        """ Callback function that processes LiDAR data and updates the plot. """
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))  # Compute angles in radians
        distances = np.array(msg.ranges)  # Convert to NumPy array

        # Convert polar (angle, distance) to Cartesian (x, y)
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)

        # Update the plot with new scan data
        self.scatter.set_offsets(np.c_[x, y])  # Update scatter plot points
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.scatter(0, 0, color='red', s=100, label="Ergocub")

        # Log some info
        self.get_logger().info(f'Received {len(msg.ranges)} LiDAR points')
        print(f'Min Range: {np.min(distances):.2f} m, Max Range: {np.max(distances):.2f} m')

    def run(self):
        """ Runs the ROS2 node with Matplotlib event handling. """
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)  # Allow ROS processing
                plt.pause(0.01)  # Keep updating the plot
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    node.run()


if __name__ == '__main__':
    main()

