import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
from transforms3d.euler import quat2euler



class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('imu_visualizer')

        self.subscription = self.create_subscription(
            Imu,
            '/imu_head_data',  # Topic name
            self.imu_callback,
            10
        )

        # Buffers
        self.maxlen = 100
        self.time = deque(maxlen=self.maxlen)
        self.lin_acc = {axis: deque(maxlen=self.maxlen) for axis in 'xyz'}
        self.ang_vel = {axis: deque(maxlen=self.maxlen) for axis in 'xyz'}
        self.orientation = {axis: deque(maxlen=self.maxlen) for axis in ['roll', 'pitch', 'yaw']}

        # Plotting
        self.fig, self.axs = plt.subplots(3, 1, figsize=(10, 9))
        self.fig.tight_layout()

        self.lines = {
            'lin_acc': {axis: self.axs[0].plot([], [], label=f'Acc {axis}')[0] for axis in 'xyz'},
            'ang_vel': {axis: self.axs[1].plot([], [], label=f'Gyro {axis}')[0] for axis in 'xyz'},
            'orient': {axis: self.axs[2].plot([], [], label=f'{axis.title()}')[0] for axis in ['roll', 'pitch', 'yaw']}
        }

        for ax in self.axs:
            ax.legend()
            ax.set_xlim(0, self.maxlen)
            ax.set_ylim(-10, 10)

        self.axs[0].set_title('Linear Acceleration (m/s²)')
        self.axs[1].set_title('Angular Velocity (rad/s)')
        self.axs[2].set_title('Orientation (Roll, Pitch, Yaw) in degrees')

        plt.ion()
        plt.show()

        self.counter = 0

    def imu_callback(self, msg: Imu):
        self.counter += 1
        self.time.append(self.counter)

        # Acceleration
        self.lin_acc['x'].append(msg.linear_acceleration.x)
        self.lin_acc['y'].append(msg.linear_acceleration.y)
        self.lin_acc['z'].append(msg.linear_acceleration.z)

        # Angular velocity
        self.ang_vel['x'].append(msg.angular_velocity.x)
        self.ang_vel['y'].append(msg.angular_velocity.y)
        self.ang_vel['z'].append(msg.angular_velocity.z)

        # Orientation
        q = msg.orientation
        quat = [q.w, q.x, q.y, q.z]
        try:
            roll, pitch, yaw = quat2euler(quat, axes='sxyz')
            self.orientation['roll'].append(np.degrees(roll))
            self.orientation['pitch'].append(np.degrees(pitch))
            self.orientation['yaw'].append(np.degrees(yaw))
        except Exception as e:
            self.get_logger().warn(f"Failed to convert orientation: {e}")
            self.orientation['roll'].append(0.0)
            self.orientation['pitch'].append(0.0)
            self.orientation['yaw'].append(0.0)

        # Update plots
        for axis in 'xyz':
            self.lines['lin_acc'][axis].set_data(self.time, self.lin_acc[axis])
            self.lines['ang_vel'][axis].set_data(self.time, self.ang_vel[axis])
        for axis in ['roll', 'pitch', 'yaw']:
            self.lines['orient'][axis].set_data(self.time, self.orientation[axis])

        for ax in self.axs:
            ax.set_xlim(max(0, self.counter - self.maxlen), self.counter)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                plt.pause(0.01)
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = IMUSubscriber()
    node.run()


if __name__ == '__main__':
    main()

