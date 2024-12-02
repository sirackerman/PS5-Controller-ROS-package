# PS5 Controller ROS Package

This ROS package enables control of ROS-compatible robots using a PS5 DualSense controller. It features both analog stick and D-pad control modes, with customizable speed settings and a turbo mode.

## Features

- D-pad control mode for precise digital input
- Turbo mode for faster movement
- Compatible with any ROS robot publishing to /cmd_vel topic

## Prerequisites

### Hardware
- PlayStation 5 DualSense Controller
- USB-C cable for wired connection

### Software
- Ubuntu 18.04 or newer

## Installation

1. First, install the required ROS packages:
```bash
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-joy ros-${ROS_DISTRO}-teleop-twist-joy
```

2. Clone this repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/sirackerman/ps5_teleop.git
```

3. Build the package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Connecting the PS5 Controller
1. Connect your PS5 controller to your computer using a USB-C cable
2. Verify the connection:
```bash
ls /dev/input/js*
```
3. Set permissions for the controller:
```bash
sudo chmod a+rw /dev/input/js*
```

### Testing the Connection
```bash
# In terminal 1
roscore

# In terminal 2
rosrun joy joy_node

# In terminal 3
rostopic echo /joy
```
Press buttons on the controller - you should see the values change in terminal 3.

## Usage

1. Launch your robot's main launch file (example with Turtlebot3):
```bash
export TURTLEBOT3_MODEL=burger  # Replace with your robot model
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
2. Launch the PS5 controller node:
```bash
roslaunch ps5_teleop ps5_teleop.launch
```

### Default Control Scheme

#### D-pad Mode (Default)
- Hold L1 (enable button)
- Up Arrow: Move Forward
- Down Arrow: Move Backward
- Left Arrow: Turn Left
- Right Arrow: Turn Right
- Hold L2 while moving: Turbo Mode (faster movement)

## Customization and troubleshooting
Check customization_and_troubleshooting.txt file.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
