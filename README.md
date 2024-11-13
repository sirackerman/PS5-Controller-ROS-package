# PS5 Controller ROS Package

This ROS package enables control of ROS-compatible robots using a PS5 DualSense controller. It features both analog stick and D-pad control modes, with customizable speed settings and a turbo mode.

## Features

- D-pad control mode for precise digital input
- Configurable speed settings
- Turbo mode for faster movement
- Custom input filtering for stable movement
- Compatible with any ROS robot publishing to /cmd_vel topic
- Support for both linear and angular movement
- Support for both USB-C and Bluetooth connections

## Prerequisites

### Hardware
- PlayStation 5 DualSense Controller
- USB-C cable for wired connection
  OR
- Bluetooth adapter (if not built into your computer) for wireless connection
- ROS-compatible robot that subscribes to /cmd_vel topic

### Software
- Ubuntu 18.04 or newer
- ROS Melodic or newer
- Required ROS packages:
  - joy
  - geometry_msgs
  - sensor_msgs
  - teleop_twist_joy

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

### Option 1: USB-C Connection (Recommended for lowest latency)
1. Connect your PS5 controller to your computer using a USB-C cable
2. Verify the connection:
```bash
ls /dev/input/js*
```
3. Set permissions for the controller:
```bash
sudo chmod a+rw /dev/input/js*
```

### Option 2: Bluetooth Connection
1. Put your PS5 controller in pairing mode:
   - Press and hold the PS button and Share button until the light bar starts flashing
   
2. Connect the controller via Bluetooth:
   - Open Ubuntu's Bluetooth settings
   - Click on "Wireless Controller" when it appears
   - Wait for the connection to be established

3. Verify the connection:
```bash
ls /dev/input/js0
```

### Testing the Connection
Regardless of connection method, you can test if the controller is working:
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

## Customization

### Speed Settings
You can modify the speed settings in the launch file `~/catkin_ws/src/ps5_teleop/launch/ps5_teleop.launch`:

```xml
<param name="scale_linear" value="0.3" />     <!-- Normal linear speed -->
<param name="scale_angular" value="0.6" />    <!-- Normal angular speed -->
<param name="scale_linear_turbo" value="0.6" />  <!-- Turbo linear speed -->
```

### Button Mapping
The default button mappings for the PS5 controller are:
```
D-pad:
- Up arrow: axis 7 value 1
- Down arrow: axis 7 value -1
- Left arrow: axis 6 value 1
- Right arrow: axis 6 value -1

Buttons:
- L1 (Enable): button 4
- L2 (Turbo): button 6

Other Buttons (for reference):
- L3: button 10
- R1: button 5
- R2: button 7
- R3: button 11
- Triangle: button 3
- Square: button 0
- Cross: button 1
- Circle: button 2
- Share: button 8
- Options: button 9
- PS button: button 12
- Touchpad: button 13
```

If your controller has different mappings, you can check them using:
```bash
rosrun joy joy_node
rostopic echo /joy
```

Then update the parameters in the launch file accordingly.

## Troubleshooting

### Controller Not Detected
1. For USB-C connection:
   - Try unplugging and replugging the USB-C cable
   - Try a different USB port
   - Test the cable with another device to ensure it's working
   - Check if the controller is detected:
     ```bash
     ls -l /dev/input/js*
     ```

2. For Bluetooth connection:
   - Try removing the controller from Bluetooth devices and re-pairing
   - Ensure the controller is charged
   - Check if other Bluetooth devices are working

3. Verify permissions:
```bash
sudo chmod a+rw /dev/input/js*
```

4. Check if the joy node is receiving input:
```bash
rostopic echo /joy
```

### Robot Not Moving
1. Verify cmd_vel messages are being published:
```bash
rostopic echo /cmd_vel
```

2. Check that your robot is subscribing to /cmd_vel:
```bash
rostopic info /cmd_vel
```

3. Ensure the enable button (L1) is pressed while trying to move

### Input Lag
- If experiencing input lag with Bluetooth, try switching to a USB-C connection
- Make sure your USB port is USB 3.0 or higher for best performance
- Check CPU usage to ensure your system isn't overloaded



## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments
- ROS Joy package developers
- Teleop Twist Joy package developers
- The ROS community
