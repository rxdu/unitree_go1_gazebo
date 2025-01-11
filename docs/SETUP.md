# Gazebo Simulation Setup

## 1. Setup the robot model

You first need to define the robot joints and links in an urdf or xacro file. You can verify the robot joint setup with
`joint_state_publisher_gui` and `robot_state_publisher`:

```bash
ros2 launch unitree_go1_gazebo description.launch.py
```

You should see the robot joints and robot model displayed in the RViz window.

![](screenshots/go1_urdf.png)

Besides the information of the robot itself, you also need to define the robot's sensors, controllers, and plugins for
the simulation.
