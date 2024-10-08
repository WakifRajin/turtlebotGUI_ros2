# TurtleBot GUI for ROS2

This project provides a graphical user interface (GUI) for controlling a TurtleBot using ROS2 (Robot Operating System 2). The GUI allows users to visualize the robot's odometry, velocity data, and control it via joystick or buttons.

## Features

- Real-time odometry and velocity display
- Joystick/button controls for robot movement
- Built using Python and ROS2

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/WakifRajin/turtlebotGUI_ros2.git
   cd turtlebotGUI_ros2
2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
3. Build the package
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select gui_bot
   source install/setup.bash
4. Run the GUI
   ```bash
   ros2 run gui_bot gui
5. Launch turtlebot3
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
