import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ROS2Controller(Node):
    def __init__(self):
        super().__init__('qt_cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/ergocub/cmd_vel', 10)

    def publish_velocity(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        print(f"Sent: linear={linear:.2f}, angular={angular:.2f}")

class MainWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle('Robot Controller')
        self.setGeometry(100, 100, 300, 250)

        layout = QVBoxLayout()

        self.label = QLabel("Use buttons or arrow keys", self)
        layout.addWidget(self.label)

        self.btn_fwd = QPushButton("↑ Forward")
        self.btn_back = QPushButton("↓ Backward")
        self.btn_left = QPushButton("← Left")
        self.btn_right = QPushButton("→ Right")
        self.btn_stop = QPushButton("■ Stop")

        layout.addWidget(self.btn_fwd)
        layout.addWidget(self.btn_back)
        layout.addWidget(self.btn_left)
        layout.addWidget(self.btn_right)
        layout.addWidget(self.btn_stop)

        self.setLayout(layout)

        self.btn_fwd.clicked.connect(lambda: self.ros_node.publish_velocity(0.5, 0.0))
        self.btn_back.clicked.connect(lambda: self.ros_node.publish_velocity(-0.5, 0.0))
        self.btn_left.clicked.connect(lambda: self.ros_node.publish_velocity(0.0, 1.0))
        self.btn_right.clicked.connect(lambda: self.ros_node.publish_velocity(0.0, -1.0))
        self.btn_stop.clicked.connect(lambda: self.ros_node.publish_velocity(0.0, 0.0))

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_W:
            self.ros_node.publish_velocity(0.5, 0.0)
        elif key == Qt.Key_S:
            self.ros_node.publish_velocity(-0.5, 0.0)
        elif key == Qt.Key_A:
            self.ros_node.publish_velocity(0.0, 2.0)
        elif key == Qt.Key_D:
            self.ros_node.publish_velocity(0.0, -2.0)


    def keyReleaseEvent(self, event):
        key = event.key()
        if key == Qt.Key_W:
            self.ros_node.publish_velocity(0.0, 0.0)
        elif key == Qt.Key_S:
            self.ros_node.publish_velocity(0.0, 0.0)
        elif key == Qt.Key_A:
            self.ros_node.publish_velocity(0.0, 0.0)
        elif key == Qt.Key_D:
            self.ros_node.publish_velocity(0.0, 0.0)


def main():
    rclpy.init()
    ros_node = ROS2Controller()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    try:
        sys.exit(app.exec_())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
