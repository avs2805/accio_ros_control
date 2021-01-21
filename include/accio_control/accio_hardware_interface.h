#ifndef ROS_CONTROL__ACCIO_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ACCIO_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
// #include <acciocpp/accio.h>
// #include <accio_control/accio_ha

    class AccioHardwareInterface: public hardware_interface::RobotHW
    {
    public:
        // Constructor
        AccioHardwareInterface(ros::NodeHandle &nh);
        // Destructor
        ~AccioHardwareInterface();
        // Init, Update, Read, Write
        void init();
        void update(const ros::TimerEvent &e);
        void read();
        void write(ros::Duration elapsed_time);

    protected:
        // Basic Interfaces
        // Joint state interface provides the current joint state of the robot
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;
        hardware_interface::EffortJointInterface effort_joint_interface_;

        // Saturation and Limits Interface
        joint_limits_interface::EffortJointSaturationInterface effort_joint_saturation_interface_;
        joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_limits_interface_;
        joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
        joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;
        joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
        joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limits_interface_;

        // Shared memory
        int num_joints_;
        int joint_mode_; // position, velocity, or effort
        std::vector<std::string> joint_names_;
        std::vector<int> joint_types_;
        std::vector<double> joint_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;
        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocity_command_;
        std::vector<double> joint_effort_command_;
        std::vector<double> joint_lower_limits_;
        std::vector<double> joint_upper_limits_;
        std::vector<double> joint_effort_limits_;

        ros::NodeHandle node_handle_;
        ros::Timer my_control_loop_;
        ros::Timer non_realtime_loop_;
        ros::Duration control_period_;
        ros::Duration elapsed_time_;

        double loop_hz_;

        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

        double p_error_, v_error_, e_error_;
    };

#endif