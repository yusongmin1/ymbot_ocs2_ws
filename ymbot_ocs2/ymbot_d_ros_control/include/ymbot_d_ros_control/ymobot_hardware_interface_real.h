#ifndef YMBOT_HW_INTERFACE_REAL
#define YMBOT_HW_INTERFACE_REAL
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_state.h>
#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h> // for sleep()
using namespace std;

// 共享内存28个浮点数 前十四个是左臂右臂状态，后十四个是左臂右臂控制 
// 初始化状态时有bug
// 初始化ros_control的关节状态为mpc的初始状态，这个两边都要改
#define joint_num 20

class YMBOTDHW : public hardware_interface::RobotHW 
{
        public:
        using Clock = std::chrono::high_resolution_clock;
        using Duration = std::chrono::duration<double>;
        YMBOTDHW(ros::NodeHandle& nh);
        ~YMBOTDHW();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);

        // 电机补偿角，与 urdf 模型的初始姿态以及实际机器人的姿态有关  左 右 躯干 ，数值需要修改
        std::vector<double>  joints_offset_angle =  
        {
            188.789, 175.6+10, 186.498, 176.276-45, 94.059, 179.951, 185.779,
            181.214, 185.905-10, 181.121, 182.807+45, 266.259, 176.874, 179.786,
            224.33-65, 170.299-(90+65), 195.925+90, 177.138,
            179.209, 176.133
        };
        //mpc初始计算时的关节角度，左臂右臂，如果机器人初始不在这个位置，会有较大加速度
        vector<double> joint_init_angle={-0.4583,0.3746,0.5241,0.1204,-0.5028,-0.5396,-0.2679,
                                        0.4583,-0.3746,-0.5241,-0.1204,0.5027,0.5396,0.2679};
        hardware_interface::JointStateInterface     joint_state_interface_;
        hardware_interface::PositionJointInterface  position_joint_interface_;
        hardware_interface::EffortJointInterface    effort_joint_interface_;

        std::vector<string> joint_name_;  
        vector<double> joint_position_;
        vector<double> joint_velocity_;
        vector<double> joint_effort_;
        vector<double> joint_position_command_;
        vector<double> joint_effort_command_;
        vector<double> joint_velocity_command_;

        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

        double cycleTimeErrorThreshold_{}, loopHz_{};
        std::thread loopThread_;
        ros::Duration elapsedTime_;
        int shm_id ;
        float* shared_mem ;

};


#endif