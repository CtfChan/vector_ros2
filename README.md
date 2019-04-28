# ROS2 Driver for Anki Vector 
This driver has only been tested on Ubuntu 18.04. This is still a work in progress.

1. Setup Anki Vector for the SDK as in the documentation.
2. Clone this repo
3. Have fun


# Testing movement
```console
$ ros2 topic pub -r 10 vector/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
$ ros2 topic pub -r 10 vector/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

```console
$ ros2 service call /vector/lift_height vector_ros2_interfaces/LiftHeight '{desired_height: 0.0}'

$ ros2 service call /vector/head_angle vector_ros2_interfaces/HeadAngle '{desired_angle: 11}'

```


# Dependencies
vector_ros2_interfaces



```console
$ sudo apt-get install ros-crystal-vision-opencv
$ sudo apt-get install ros-crystal-cv-bridge
```



<!-- ```console
$ mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
$ git clone https://github.com/CtfChan/ros2_template_py.git

$ colcon build --symlink-install
$ source ~/ros2_ws/install/setup.bash && source ~/ros2_ws/install/local_setup.bash
$ ros2 run ros2_template_py demo
``` -->