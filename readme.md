# ros2-pick_and_place

This package is designed to run a pick-and-place simulation using a 6-DOF robot. (Note: This is a work in progress.)

I have utilized an existing Doosan robot URDF and integrated a gripper with the necessary ROS2 configurations. The robot is capable of executing movements by receiving tasks via an action service.

The primary objective of this project was to implement the pick-and-place functionality using MoveIt. Unfortunately, due to MoveIt2 not being fully supported in ROS Foxy, my attempts to implement it on Ubuntu 20.04 were unsuccessful. I also explored using MoveIt within a Docker container with shared resources, but time constraints prevented further investigation.

However, I successfully achieved robot movement through forward kinematics by providing joint trajectories via the action service.

Current Issues:
The gripper needs to be fully configured with the appropriate Gazebo parameters. As this is not yet complete, the robot experiences instability and breaks when attempting to pick up the box. This issue needs to be resolved by accurately adjusting the Gazebo parameters.

## Running the code
Build the workspace
```
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone git@github.com:janaardhanan/ros2-pick_and_place.git
cd ..
colcon build
```
run the gazebo simulation:
```
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_doosan_pkg my_doosan_gazebo_controller.launch.py
```

in another terminal:
```
cd ~/ros2_ws
source install/setup.bash
ros2 run my_doosan_pkg trajectory_points_act_server 
```