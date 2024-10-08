import sys
from PyQt6.QtWidgets import (QApplication, QWidget, QLabel, QVBoxLayout, QPushButton, QGridLayout)
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QKeyEvent
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.update_odometry, 10)
        self.current_velocity = Twist()

        # Initialize GUI
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("Robot Control")

        # Create layout
        self.layout = QVBoxLayout()

        # Odometry display
        self.odom_label = QLabel("Odometry: x=0, y=0")
        self.layout.addWidget(self.odom_label)

        # Velocity display
        self.vel_label = QLabel("Velocity: linear=0, angular=0")
        self.layout.addWidget(self.vel_label)

        # Control buttons layout (WASD style)
        button_layout = QGridLayout()

        self.forward_btn = QPushButton("W")
        self.forward_btn.clicked.connect(self.move_forward)
        button_layout.addWidget(self.forward_btn, 0, 1)

        self.left_btn = QPushButton("A")
        self.left_btn.clicked.connect(self.turn_left)
        button_layout.addWidget(self.left_btn, 1, 0)

        self.stop_btn = QPushButton("S")
        self.stop_btn.clicked.connect(self.stop_robot)
        button_layout.addWidget(self.stop_btn, 1, 1)

        self.right_btn = QPushButton("D")
        self.right_btn.clicked.connect(self.turn_right)
        button_layout.addWidget(self.right_btn, 1, 2)

        self.backward_btn = QPushButton("X")
        self.backward_btn.clicked.connect(self.move_backward)
        button_layout.addWidget(self.backward_btn, 2, 1)

        self.layout.addLayout(button_layout)

        # Set layout and window size
        self.window.setLayout(self.layout)
        self.window.resize(400, 200)

        # Start the GUI update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # Update every 100ms

        # Enable keyboard focus
        self.window.setFocusPolicy(self.window.focusPolicy().StrongFocus)

    def move_forward(self):
        self.current_velocity.linear.x = 0.2
        self.publisher.publish(self.current_velocity)

    def move_backward(self):
        self.current_velocity.linear.x = -0.2
        self.publisher.publish(self.current_velocity)

    def turn_left(self):
        self.current_velocity.angular.z = 0.2
        self.publisher.publish(self.current_velocity)

    def turn_right(self):
        self.current_velocity.angular.z = -0.2
        self.publisher.publish(self.current_velocity)

    def stop_robot(self):
        self.current_velocity = Twist()  # Reset the velocity
        self.publisher.publish(self.current_velocity)

    def update_odometry(self, msg):
        position = msg.pose.pose.position
        velocity = msg.twist.twist
        self.odom_label.setText(f"Odometry: x={position.x:.2f}, y={position.y:.2f}")
        self.vel_label.setText(f"Velocity: linear={velocity.linear.x:.2f}, angular={velocity.angular.z:.2f}")

    def update_gui(self):
        # This function will be called every 100ms
        pass

    # Override the key press event to allow keyboard control
    def keyPressEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key.Key_W:
            self.move_forward()
        elif event.key() == Qt.Key.Key_S:
            self.stop_robot()
        elif event.key() == Qt.Key.Key_A:
            self.turn_left()
        elif event.key() == Qt.Key.Key_D:
            self.turn_right()
        elif event.key() == Qt.Key.Key_X:
            self.move_backward()

    def run(self):
        self.window.show()
        sys.exit(self.app.exec())

def main(args=None):
    rclpy.init(args=args)
    robot_control = RobotControl()
    robot_control.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
