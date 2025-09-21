# Differential-Drive Sensor Fusion & Mapping

This project simulates a **differential drive robot** in a Gazebo apartment-like environment to study **odometry**, **trajectory tracking**, and **sensor fusion**. The robot is equipped with **LiDAR** and **wheel encoders**, uses **PD control** to follow walls, completes a loop, and compares **wheel odometry** with **Kalman-filtered trajectories**.

---

## Environment Setup

- **ROS 2 Distro**: Foxy Fitzroy
- **Gazebo Version**: 11.11.0
- **RViz2**: Installed
- **Colcon**: Installed
- **Python Version**: ≥ 3.8

---

## Dependencies

### ROS 2 Packages

| Package              | Purpose                        |
| -------------------- | ------------------------------ |
| `geometry_msgs`      | Twist, Pose messages           |
| `sensor_msgs`        | LaserScan, IMU messages        |
| `nav_msgs`           | Odometry messages              |
| `gazebo_msgs`        | ModelStates for ground truth   |
| `rclpy`              | ROS 2 Python client library    |
| `tf_transformations` | Quaternion ↔ Euler conversions |
| `launch_ros`         | Launching ROS 2 nodes          |

### Python Libraries

| Library      | Purpose                              |
| ------------ | ------------------------------------ |
| `numpy`      | Numerical computations               |
| `matplotlib` | Plotting trajectories and distances  |
| `csv`        | Logging distance and trajectory data |
| `os`         | File path management                 |
| `datetime`   | Timestamping data                    |

Install Python dependencies:

```bash
pip3 install numpy matplotlib
```
## Setup Instructions

To installs ROS 2 Foxy, Gazebo 11, required ROS 2 packages, and Python dependencies.  

chmod +x setup.sh
./setup.sh


## Steps to launch simulation

# Go to your workspace
cd ~/ws_mobile

# Clean previous builds (optional but recommended)
rm -rf build/ install/ log/

# Build all packages
colcon build

# Source the workspace
source install/setup.bash

## Run Wall-Following Algorithm
cd ~/ws_mobile/wall_follower_robot/wall_follower_robot

# Run the Python node
python3 wall_following_robot.py
