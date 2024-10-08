import tkinter as tk
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
        self.root = tk.Tk()
        self.root.title("Robot Control")

        # Odometry display
        self.odom_label = tk.Label(self.root, text="Odometry: ")
        self.odom_label.pack()

        # Velocity display
        self.vel_label = tk.Label(self.root, text="Velocity: ")
        self.vel_label.pack()

        # Buttons for control
        self.forward_btn = tk.Button(self.root, text="Move Forward", command=self.move_forward)
        self.forward_btn.pack()

        self.backward_btn = tk.Button(self.root, text="Move Backward", command=self.move_backward)
        self.backward_btn.pack()

        self.left_btn = tk.Button(self.root, text="Turn Left", command=self.turn_left)
        self.left_btn.pack()

        self.right_btn = tk.Button(self.root, text="Turn Right", command=self.turn_right)
        self.right_btn.pack()

        self.stop_btn = tk.Button(self.root, text="Stop", command=self.stop_robot)
        self.stop_btn.pack()

        self.update_gui()

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
        self.current_velocity = Twist()  
        self.publisher.publish(self.current_velocity)

    def update_odometry(self, msg):
        position = msg.pose.pose.position
        velocity = msg.twist.twist
        self.odom_label.config(text=f"Odometry: x={position.x}, y={position.y}")
        self.vel_label.config(text=f"Velocity: linear={velocity.linear.x}, angular={velocity.angular.z}")

    def update_gui(self):
        self.root.update()
        self.root.after(100, self.update_gui)  

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    robot_control = RobotControl()
    robot_control.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
