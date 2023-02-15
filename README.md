# my learning about a simple single motor using velocity control ros

## Installation

```
vcs import . < my.repos
```

## usage

### Sim

```
roslaunch motor_description gazebo.launch
```

![gazebo](https://github.com/JosefGst/hw_interface_learning/blob/master/assets/gazebo.png)

send a velocity command to the motor

```
rostopic pub --once /motor_velocity_controller/command std_msgs/Float64 "data: 0.50"
```

### real

```
roslaunch motor_interface check_velocity_controller.launch
```

![rviz-motor](https://github.com/JosefGst/hw_interface_learning/blob/master/assets/rviz-motor.png)

send a command to rotate once per 10s

```
rostopic pub --once /motor_velocity_controller/command std_msgs/Float64 "data: -0.6283"
```

## resources:

https://www.rosroboticslearning.com/ros-control  
https://github.com/bandasaikrishna/ros_control_example  
https://github.com/PickNikRobotics/ros_control_boilerplate
