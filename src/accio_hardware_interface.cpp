#include <accio_control/accio_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <std_msgs/Float32.h>

AccioHardwareInterface::AccioHardwareInterface(ros::NodeHandle &nh) : node_handle_(nh)
{
    // Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();

    // Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, node_handle_));

    //Set the frequency of the control loop.
    // loop_hz_ = 10;
    node_handle_.param("loop_hz", loop_hz_, 0.1);
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);

    my_control_loop_ = nh.createTimer(update_freq, &AccioHardwareInterface::update, this);

    lWheelSub = node_handle_.subscribe("lWheelTicks", 10, &AccioHardwareInterface::lwheel_ticks_cb, this);
    rWheelSub = node_handle_.subscribe("rWheelTicks", 10, &AccioHardwareInterface::rwheel_ticks_cb, this);

    // Initialize publishers and subscribers
    left_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("accio/left_wheel_vel", 1);
    right_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("accio/right_wheel_vel", 1);


} // constructor

AccioHardwareInterface::~AccioHardwareInterface(){}; // descructor

void AccioHardwareInterface::init()
{
    // Get joint names
    node_handle_.getParam("joints", joint_names_);
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
        // hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
        // effort_joint_interface_.registerHandle(jointEffortHandle);

    // Create velocity joint interface
	hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
    velocity_joint_interface_.registerHandle(jointVelocityHandle);

        // // Create Joint Limit interfaces
        // joint_limits_interface::JointLimits limits;
        // // joint_limits_interface::SoftJointLimits softLimits;
        // joint_limits_interface::getJointLimits(joint_names_[i], node_handle_, limits);
        // joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle(jointEffortHandle, limits);
        // effort_joint_saturation_interface_.registerHandle(jointLimitsHandle);

    } // end for loop after creating all handles
      // Register all joints interfaces
    registerInterface(&joint_state_interface_);
    // registerInterface(&position_joint_interface_);
    registerInterface(&velocity_joint_interface_);
    // registerInterface(&effort_joint_saturation_interface_);
    // registerInterface(&position_joint_saturation_interface_);

    TICKS_TO_RADS = 2 * M_PI / 300.8;
    _curr_lwheel_count,_curr_rwheel_count  = 0;
    _delta_l_enc, _delta_r_enc = 0;
    _prev_left_encoder, _prev_right_encoder = 0;
    ang_left, ang_right = 0.0;

} // init

void AccioHardwareInterface::update(const ros::TimerEvent &e)
{
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read(elapsed_time_);
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
} // update

void AccioHardwareInterface::read(ros::Duration elapsed_time)
{
    double ang_distance_left = ang_left;
    double ang_distance_right = ang_right;
    joint_position_[0] += ang_distance_left;
    joint_velocity_[0] += ang_distance_left / elapsed_time.toSec();
    joint_position_[1] += ang_distance_right;
    joint_velocity_[1] += ang_distance_right / elapsed_time.toSec();
} // read

void AccioHardwareInterface::write(ros::Duration elapsed_time)
{
    double diff_ang_speed_left = joint_effort_command_[0];
    double diff_ang_speed_right = joint_effort_command_[1];
    effort_joint_saturation_interface_.enforceLimits(elapsed_time);
    // Publish results
    std_msgs::Float32 left_wheel_vel_msg;
    std_msgs::Float32 right_wheel_vel_msg;
    left_wheel_vel_msg.data = diff_ang_speed_left;
    right_wheel_vel_msg.data = diff_ang_speed_right;
    left_wheel_vel_pub_.publish(left_wheel_vel_msg);
    right_wheel_vel_pub_.publish(right_wheel_vel_msg);
}

void AccioHardwareInterface::lwheel_ticks_cb(const std_msgs::Int32::ConstPtr &lwheel_ticks_data)
{
    _curr_lwheel_count = lwheel_ticks_data->data;
    _delta_l_enc = _curr_lwheel_count - _prev_left_encoder;
    ang_left = TICKS_TO_RADS * _delta_l_enc;
    _prev_left_encoder = _curr_lwheel_count;
}

void AccioHardwareInterface::rwheel_ticks_cb(const std_msgs::Int32::ConstPtr &rwheel_ticks_data)
{
    _curr_rwheel_count = rwheel_ticks_data->data;
    _delta_r_enc = _curr_rwheel_count - _prev_right_encoder;
    ang_right = TICKS_TO_RADS * _delta_r_enc;
    _prev_right_encoder = _curr_rwheel_count;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "accio_hardware_interface");
    // ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    // nh.setCallbackQueue(&ros_queue);
    AccioHardwareInterface AHI(nh);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}