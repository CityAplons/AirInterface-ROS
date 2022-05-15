# DroneArm control algorithms

> Environment was created and tested on Debian Buster for Raspberry Pi 4 (Raspbian64)
> ROS Noetic

## Installation

### 0.Packet dependencies

- [mavros](https://docs.px4.io/master/en/ros/mavros_installation.html)
- [rosbridge-server](https://github.com/RobotWebTools/rosbridge_suite/tree/ros1)
- [vicon_bridge](https://github.com/ethz-asl/vicon_bridge)
- [usb_cam](https://github.com/ros-drivers/usb_cam)

> NOTE: vicon_bridge is available on GitHub. Please, build it from source. See step 3 below.

```bash
sudo apt install ros-noetic-rosbridge-server ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-usb-cam
```

### 1.Load PX4 configuration parameters

Using QGroundControl load `drone_arms.params` if PixhawkAP version < 1.12 or `drone_arms_new_fw.params` otherwise located in `params` folder. This parameters includes PID calibration and necessary mappings for MoCap and mavros,
such as enabled TELEM2 port with 92100 baud rate and parameters described in [PX4 MoCap article](https://docs.px4.io/master/en/ros/external_position_estimation.html).

### 2.Verify your Network parameters, Vicon IP & host IP in `drone_arm.launch`

Additionally check the serial interface for arm controller in `scripts/arm_translator.py`

### 3.Build ROS package in your workspace

```bash
# It is recommended to assemble project in an isolated ROS environment
mkdir ~/airinterface_ws && cd ~/airinterface_ws
# Clone the package into ROS workspace of the drone computer
# NOTE: execute these commands in the workspace's root directory
git clone -b release https://github.com/CityAplons/AirInterface-ROS.git src/airinterface_drone
git clone https://github.com/ethz-asl/vicon_bridge src/vicon_bridge
# Build the workspace
sudo rosdep init 
rosdep update
catkin build
# Pass installation to your bash environment
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
```

> If `catkin build` unavailable, `catkin_make` should works.

## Script Execution

### For the actual testing environment

```bash
roslaunch airinterface drone_arm.launch # Drone + Vicon + Camera + ROS Bridge + Controls 
```

### For simulation

Firstly, you should run the simulation environment (jmavsim or gazebo, see [PX4 Docs](https://docs.px4.io/master/en/simulation/)). Then to launch controls, you may pass this command:

```bash
roslaunch airinterface drone_sim.launch # Drone + ROS Bridge + Controls 
```

### After, you could run Unity with ROS# stack

or check Unity controls with a simple keyboard teleoperation script:

```bash
rosrun airinterface keyboard_pose
```
