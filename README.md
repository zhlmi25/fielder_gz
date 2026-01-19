# ROS2 Project Setup Guide

This guide will help you set up the necessary dependencies for this ROS2 project.

## Prerequisites

Make sure you have **ROS2 Jazzy** installed on your system. If not, follow the [official ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation.html).

## 1. Install Package Dependencies

Install the required ROS2 packages for this project:

```bash
# Install joint_state_publisher and joint_state_publisher_gui
sudo apt install ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui

# Install xacro
sudo apt install ros-jazzy-xacro

# Install SLAM Toolbox
sudo apt install ros-jazzy-slam-toolbox

# Install Nav2
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Install additional useful tools
sudo apt install ros-jazzy-turtlebot3* ros-jazzy-robot-state-publisher
```

## 2. Install Gazebo for ROS2 Jazzy

ROS2 Jazzy uses **Gazebo Harmonic** (formerly Ignition Gazebo).

```bash
# Install Gazebo Harmonic
sudo apt install ros-jazzy-ros-gz

# Install Gazebo simulation packages
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-ros2-control

# Verify installation
gz sim --version
```

## Building the Workspace

After installing all dependencies, build your workspace:

```bash
# Navigate to your workspace
cd ~/your_ros2_workspace

# Install dependencies using rosdep
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Running the Project

```bash
# Launch your robot in Gazebo
ros2 launch <your_package> <your_launch_file>
```

## Troubleshooting

- If you encounter dependency issues, run: `rosdep update && rosdep install --from-paths src --ignore-src -r -y`
- Make sure you've sourced both ROS2 and your workspace: `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
- For Gazebo issues, check that the `gz sim` command works independently

## Additional Resources

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)

## License

[Add your license here]

## Contributors

[Add contributors here]
