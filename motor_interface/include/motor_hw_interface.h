#ifndef MOTOR_INTERFACE
#define MOTOR_INTERFACE

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
// #include <joint_limits_interface/joint_limits.h>
// #include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include "rmdx.h"

#define DEG_TO_RAD 0.01745329251
#define RAD_TO_DEG 57.2957795131

class MyMotor : public hardware_interface::RobotHW
{
public:
    MyMotor(ros::NodeHandle &nh);
    ~MyMotor();
    void init();
    void update(const ros::TimerEvent &e);
    void read();
    void write(ros::Duration elapsed_time);
    RMDX motor;

protected:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    // joint_limits_interface::JointLimits limits;
    // joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
    // joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;

    double joint_position_;
    double joint_velocity_;
    double joint_effort_;
    double joint_velocity_command_;
    double joint_position_command_;
    double joint_effort_command_;

    ros::NodeHandle nh_;
    ros::Timer my_control_loop_;
    ros::Duration elapsed_time_;
    double loop_hz_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

#endif