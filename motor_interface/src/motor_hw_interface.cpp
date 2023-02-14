#include "motor_hw_interface.h"

MyMotor::MyMotor(ros::NodeHandle &nh) : nh_(nh)
{

    // Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();

    // Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

    // Set the frequency of the control loop.
    loop_hz_ = 1;
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);

    // Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &MyMotor::update, this);
}
MyMotor::~MyMotor()
{
}

void MyMotor::init()
{
    // motor.beginn("/dev/ttyUSB0", 115200, 0x00);

    // Create joint_state_interface for JointA
    hardware_interface::JointStateHandle jointStateHandleA("shaft_output_joint", &joint_position_, &joint_velocity_, &joint_effort_);
    joint_state_interface_.registerHandle(jointStateHandleA);
    // Create effort joint interface as JointA accepts effort command.
    hardware_interface::JointHandle jointVelocityHandleA(jointStateHandleA, &joint_velocity_command_);
    velocity_joint_interface_.registerHandle(jointVelocityHandleA);
    // Create Joint Limit interface for JointA
    // joint_limits_interface::getJointLimits("shaft_output_joint", nh_, limits);
    // joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandleA(jointVelocityHandleA, limits);
    // velocityJointSaturationInterface.registerHandle(jointLimitsHandleA);

    // Register all joints interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&position_joint_interface_);
    // registerInterface(&velocityJointSaturationInterface);
    // registerInterface(&positionJointSaturationInterface);
}

// This is the control loop
void MyMotor::update(const ros::TimerEvent &e)
{
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void MyMotor::read()
{
    // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to get the current joint position and/or velocity and/or effort

    // from robot.
    //  and fill JointStateHandle variables joint_position_[i], joint_velocity_[i] and joint_effort_[i]

    // motor.read_stat();
    joint_position_ = joint_velocity_command_;
    joint_velocity_ = joint_velocity_command_;
    joint_effort_ = joint_velocity_command_;
}

void MyMotor::write(ros::Duration elapsed_time)
{
    // Safety
    // velocityJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointA and JointB
    // positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointC

    // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
    // the output commands need to send are joint_velocity_command_[0] for JointA, joint_velocity_command_[1] for JointB and

    // joint_position_command_ for JointC.

    ROS_INFO("Hello %f", joint_velocity_command_);
    // motor.vel_cmd(joint_velocity_command_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_joint_hardware_interface");
    ros::NodeHandle nh;
    // ros::AsyncSpinner spinner(4);
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    MyMotor ROBOT(nh);
    // spinner.start();
    spinner.spin();
    // ros::spin();
    return 0;
}