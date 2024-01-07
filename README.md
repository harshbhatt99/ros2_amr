# ROS2 AMR (Autonomous Mobile Robot)
This project focusses on advancing the autonomous navigation capabilities of the TurtleBot Autonomous Mobile Robot (AMR) by leveraging the power of ROS2 and its navigation stack. The goal is to fine-tune and optimize navigation parameters to suit different environmental contexts, enhancing the robot's adaptability, efficiency, and safety.

### Dependencies

* ROS2 Humble
* Ubuntu 22.04

### Installing
* Install necessary turtlebot3 packages
```
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
```
* Clone the repository under src folder of your ROS workspace.
* Remove the folder "Snapshots".
* Change current directory to ROS workspace and run "colcon build".
* Source the setup.bash file in current ROS2 workspace.
```
source install/setup.bash
```

### Executing program

* Run the launch file to launch Gazebo with custom world.
```
ros2 launch ros2_amr amr_navigation.launch.py
```
* Run the navigation stack with default configuration.
```
ros2 launch ros2_amr bringup_launch.py use_sim_time:=True autostart:=True map:=/home/hb/ros2_turtle/src/ros2_amr/config/custom_world_2.yaml
```
* Run RViz with default configurations. Locate the robot and add 2D pose estimate.
```
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```
* Set 2D Nav Goal and observe the performance.
* Open RQT console for dynamic configuration of navigation parameters.

### Acknowledgement

* Turtlebot3 Packages [https://github.com/ROBOTIS-GIT/turtlebot3]



