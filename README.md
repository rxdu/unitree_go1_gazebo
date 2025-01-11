# unitree_go1_gazebo

This repository contains a Gazebo simulation setup for Unitree GO1. The simulated robot provides a similar interface to
the real robot, allowing users to easily develop programs with the simulation first and then transfer to the real robot.

* The setup follows the layout of the [ros_gz_project_template](https://github.com/gazebosim/ros_gz_project_template)
  package.
* The GO1 model is based on go1_description package from [unitree_ros](https://github.com/unitreerobotics/unitree_ros)
  package.

This repository also serves as a reference for creating a Gazebo simulation for any robot.

## Supported platform

The following platforms are supported and tested:

* Ubuntu 24.04
* ROS Jazzy
* Gazebo Harmonic

## Build the package

* Install ROS and Gazebo Harmonic

You may refer to the [official ROS installation guide](https://docs.ros.org/en/jazzy/Installation.html) for the
installation of ROS Jazzy. 

Follow the following steps to install Gazebo Harmonic:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

* Install the ROS dependencies

```bash
$ cd <directory_of_the_colcon_workspace>
$ rosdep install --from-paths src -y --ignore-src
```

## Gazebo simulation setup

You can find the detailed instructions on how to setup the Gazebo simulation in the [SETUP.md](docs/SETUP.md) document.


