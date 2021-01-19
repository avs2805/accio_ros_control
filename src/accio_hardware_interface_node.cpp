#include <accio_hardware_interface/accio_hardware_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "accio_hardware_interface");
    ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&ros_queue);
    ROBOT_hardware_interface::ROBOTHardwareInterface rhi(nh);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);
    return 0;
}