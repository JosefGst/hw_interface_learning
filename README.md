# my learning about a simple single motor using velocity control ros

## Installation

```
vcs import . < my.repos
```

## usage

```
roslaunch motor_description gazebo.launch
```

![gazebo](https://github.com/JosefGst/hw_interface_learning/blob/master/motor_description/assets/gazebo.png)

send a velocity command to the motor

```
rostopic pub --once /motor_velocity_controller/command std_msgs/Float64 "data: 0.50"
```
