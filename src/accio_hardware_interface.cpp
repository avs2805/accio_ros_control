#include <accio_control/accio_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

AccioHardwareInterface::AccioHardwareInterface(ros::NodeHandle &nh) : node_handle_(nh)
{
    // Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();

    // Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, node_handle_));

    //Set the frequency of the control loop.
    // loop_hz_ = 10;
    node_handle_.param("/accio/loop_hz", loop_hz_, 0.1);
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);

    my_control_loop_ = nh.createTimer(update_freq, &AccioHardwareInterface::update, this);
} // constructor

AccioHardwareInterface::~AccioHardwareInterface() {} // descructor

void AccioHardwareInterface::init()
{
    // Get joint names
    node_handle_.getParam("/accio/joints", joint_names_);
    num_joints_ = joint_names_.size();

    // resize vectors
    joint_velocity_.resize(num_joints_);
    joint_position_.resize(num_joints_);
    joint_effort_.resize(num_joints_);
    joint_position_command_.resize(num_joints_);
    joint_velocity_command_.resize(num_joints_);
    joint_effort_command_.resize(num_joints_);

    // intialize the controller
    for (int i = 0; i < num_joints_; ++i)
    {

        // Create joint state interfaces
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create joint effort interfaces
        hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
        effort_joint_interface_.registerHandle(jointEffortHandle);

        // Create Joint Limit interfaces
        joint_limits_interface::JointLimits limits;
        // joint_limits_interface::SoftJointLimits softLimits;
        joint_limits_interface::getJointLimits(joint_names_[i], node_handle_, limits);
        joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle(jointEffortHandle, limits);
        effort_joint_saturation_interface_.registerHandle(jointLimitsHandle);

    } // end for loop after creating all handles
      // Register all joints interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&effort_joint_saturation_interface_);
    registerInterface(&position_joint_saturation_interface_);

} // init

void accio_hardware_interface::update(const ros::TimerEvent &e)
{
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
} // update

void accio_hardware_interface::read()
{
    // do nothing
} // read

void accio_hardware_interface::write(ros::Duration elapsed_time)
{
    effort_joint_saturation_interface_.enforceLimits(elapsed_time);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "accio_hardware_interface");
    // ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    // nh.setCallbackQueue(&ros_queue);
    accio_hardware_interface::accio_hardware_interface ahi(nh);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();
    return 0;
}