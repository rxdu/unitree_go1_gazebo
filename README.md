# unitree_go1_gazebo

This repository contains a Gazebo simulation setup for Unitree GO1. The simulated robot provides a similar interface to
the real robot, allowing users to easily develop programs with the simulation first and then transfer to the real robot.

* The setup referred to the configurations in
  the [ros_gz_project_template](https://github.com/gazebosim/ros_gz_project_template) package.
* The GO1 model is based on go1_description package from
  the [unitree_ros](https://github.com/unitreerobotics/unitree_ros) package.
* The LegJointController is based on the LegPdController from
  the [quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control) package.

<img src="docs/screenshots/go1_gazebo.png" height="250"> <img src="docs/screenshots/go1_ft_sensor.png" height="250">

This repository also serves as a reference for creating a Gazebo (ignition) simulation for any robot. You can find
documentation of the simulation setup process in [SETUP.md](docs/SETUP.md).

## Supported platform

The following platforms are supported and tested:

* Ubuntu 24.04
* ROS Jazzy
* Gazebo Harmonic

## Build the package

* Install ROS and Gazebo Harmonic

You may refer to the [official ROS installation guide](https://docs.ros.org/en/jazzy/Installation.html) for the
installation of ROS Jazzy.

Then you can install the bundled Gazebo Harmonic with the following command:

```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

* Clone repositories

At the time of writing, the `gz_ros2_control` package doesn't include the support for the force-torque sensors. You can
use the forked version of the package that includes the support:

```bash
cd <directory_of_the_colcon_workspace>/src
git clone -b jazzy https://github.com/rxdu/gz_ros2_control.git
git clone https://github.com/rxdu/unitree_go1_gazebo.git
```

* Install the ROS dependencies

Since the robot dog has a lot of joints (3 per leg, 12 in total), the `ros2_control` framework is used for the control
interface

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

Additional dependencies can be installed with rosdep:

```bash
cd <directory_of_the_colcon_workspace>
rosdep install --from-paths src -y --ignore-src
```

* Build the workspace

```bash
colcon build --event-handlers console_direct+ --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
```

Note that the "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" option is used to generate the compile_commands.json file for the
Code Editor/IDE (e.g. Visual Studio Code, Clion).

## Launch the simulator

```bash
ros2 launch unitree_go1_gazebo gazebo.launch.py
```

## Control interface

With the above setup, you will get the following hardware interfaces for your ros2_control controllers:

```bash
$ ros2 control list_hardware_interfaces 
command interfaces
        FL_calf_joint/effort [available] [claimed]
        FL_hip_joint/effort [available] [claimed]
        FL_thigh_joint/effort [available] [claimed]
        FR_calf_joint/effort [available] [claimed]
        FR_hip_joint/effort [available] [claimed]
        FR_thigh_joint/effort [available] [claimed]
        RL_calf_joint/effort [available] [claimed]
        RL_hip_joint/effort [available] [claimed]
        RL_thigh_joint/effort [available] [claimed]
        RR_calf_joint/effort [available] [claimed]
        RR_hip_joint/effort [available] [claimed]
        RR_thigh_joint/effort [available] [claimed]
        leg_joint_controller/FL_calf_joint/effort [available] [unclaimed]
        leg_joint_controller/FL_calf_joint/kd [available] [unclaimed]
        leg_joint_controller/FL_calf_joint/kp [available] [unclaimed]
        leg_joint_controller/FL_calf_joint/position [available] [unclaimed]
        leg_joint_controller/FL_calf_joint/velocity [available] [unclaimed]
        leg_joint_controller/FL_hip_joint/effort [available] [unclaimed]
        leg_joint_controller/FL_hip_joint/kd [available] [unclaimed]
        leg_joint_controller/FL_hip_joint/kp [available] [unclaimed]
        leg_joint_controller/FL_hip_joint/position [available] [unclaimed]
        leg_joint_controller/FL_hip_joint/velocity [available] [unclaimed]
        leg_joint_controller/FL_thigh_joint/effort [available] [unclaimed]
        leg_joint_controller/FL_thigh_joint/kd [available] [unclaimed]
        leg_joint_controller/FL_thigh_joint/kp [available] [unclaimed]
        leg_joint_controller/FL_thigh_joint/position [available] [unclaimed]
        leg_joint_controller/FL_thigh_joint/velocity [available] [unclaimed]
        ...
        [similiar leg_joint_controller interfaces for other legs]
        ...
state interfaces
        FL_calf_joint/effort
        FL_calf_joint/position
        FL_calf_joint/velocity
        FL_ft_sensor/force.x
        FL_ft_sensor/force.y
        FL_ft_sensor/force.z
        FL_ft_sensor/torque.x
        FL_ft_sensor/torque.y
        FL_ft_sensor/torque.z
        FL_hip_joint/effort
        FL_hip_joint/position
        FL_hip_joint/velocity
        FL_thigh_joint/effort
        FL_thigh_joint/position
        FL_thigh_joint/velocity
        ...
        [similiar joint state interfaces for other legs]
        ...
        imu_sensor/angular_velocity.x
        imu_sensor/angular_velocity.y
        imu_sensor/angular_velocity.z
        imu_sensor/linear_acceleration.x
        imu_sensor/linear_acceleration.y
        imu_sensor/linear_acceleration.z
        imu_sensor/orientation.w
        imu_sensor/orientation.x
        imu_sensor/orientation.y
        imu_sensor/orientation.z
        fl_ft_sensor_broadcaster/FL_ft_sensor/force.x
        fl_ft_sensor_broadcaster/FL_ft_sensor/force.y
        fl_ft_sensor_broadcaster/FL_ft_sensor/force.z
        fl_ft_sensor_broadcaster/FL_ft_sensor/torque.x
        fl_ft_sensor_broadcaster/FL_ft_sensor/torque.y
        fl_ft_sensor_broadcaster/FL_ft_sensor/torque.z
        ...
        [similiar ft sensor broadcaster interfaces for other legs]
        ...
```

You may consider using the [`unitree_api_adapter`](https://github.com/rxdu/unitree_api_adapter.git) controller plugin to
expose ROS2/DDS topic interfaces (e.g. `/lowcmd`, `/lowstate` topics), with which you can test your own low-level
controller in the same way as the real robot. Please note that unless you implement your own low-level controller, the
simulated robot won't respond to the high-level commands defined in the interface.
