#include <ymbot_d_ros_control/ymobot_hardware_interface_sim.h>
#include<thread>




// using namespace Mobilemanipulate;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ymbot_d_ros_control_sim");
    ros::NodeHandle nh;

    ros::MultiThreadedSpinner spinner(2); 
    YMBOTDHW ROBOT(nh);
    
    spinner.spin();
    return 0;
}
