# ROS2 Driver for Anki Vector 
This driver has only been tested on Ubuntu 18.04. I wrote this so that I could use Vector to mop my floor and try out ROS2.
1. Setup Anki Vector for the SDK as in the documentation.
2. Clone this repo
3. Have fun with the mopper node

# Video


# Dependencies
vector_ros2_interfaces
ros-crystal-vision-opencv
ros-crystal-cv-bridge

```console
$ sudo apt-get install ros-crystal-vision-opencv
$ sudo apt-get install ros-crystal-cv-bridge
```

# Setup
```console
$ mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
$ git clone https://github.com/CtfChan/vector_ros2.git
$ git clone https://github.com/CtfChan/vector_ros2_interfaces.git 
$ cd ~/ros2_ws 
$ colcon build --symlink-install
$ source ~/ros2_ws/install/setup.bash && source ~/ros2_ws/install/local_setup.bash
```

# Running the Mopper Node
```console
# Pane 1
$ ros2 run vector_ros2 vision
# Pane 2 
$ ros2 run vector_ros2 movement
# Pane 3 
$ ros2 run vector_ros2 mopper
```

# Testing Movement Node
Testing the rostopics
```console
$ ros2 topic pub -r 10 vector/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
$ ros2 topic pub -r 10 vector/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

Testing the rosservice
```console
$ ros2 service call /vector/lift_height vector_ros2_interfaces/LiftHeight '{desired_height: 0.0}'
$ ros2 service call /vector/head_angle vector_ros2_interfaces/HeadAngle '{desired_angle: 11}'

```
