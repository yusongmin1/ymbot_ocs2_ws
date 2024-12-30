#ifndef YMBOT_HW_INTERFACE_SIM
#define YMBOT_HW_INTERFACE_SIM
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
        std::vector<double> joints_offset_angle = {1.82373, 180.374, 182.23,  183.763, 190.074, 171.002, 182.214,
                                                148.821, 26.9604, 170.145, 7.196,   1.19751, 242.721, 176.638,
                                                184.922, 181.434, 212.086, 174.133, 186.279, 183.406};
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

YMBOTDHW::YMBOTDHW(ros::NodeHandle& nh):nh_(nh){
    
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=50;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    
    //打开共享内存，前七个是状态，后七个是控制
	shm_id = shmget((key_t)1111, 28* sizeof(float), 0666 | IPC_CREAT);
    if (shm_id == -1) {
        std::cerr << "Failed to access shared memory" << std::endl;
        return ;
    }
    // 将共享内存映射到进程地址空间
    shared_mem = (float*)shmat(shm_id, NULL, 0);
    if (shared_mem == (float*)-1) {
        std::cerr << "Failed to attach shared memory" << std::endl;
        return ;
    }

    
    std::cout<<"###########################################\n";
    std::cout<<"#########load init state from yaml#########\n";
    std::cout<<"###########################################\n";
    std::map<std::string, double> initial_positions;
    // 获取initial_positions参数
    if (ros::param::get("initial_positions", initial_positions)) {
        std::cout << "Initial Positions: \n";
        int num=0;
        for (const auto& pair : initial_positions) {
            joint_init_angle[num]=(double)pair.second;
            ROS_INFO("%s: %f", pair.first.c_str(), pair.second);
            num++;
        }
        std::cout << std::endl;
    } 
    else {
        ROS_ERROR("Failed to get joint_init_angle from parameter server.");
        return;
    }

    //初始化位置和控制指令，使其均为初始状态，mpc的初始状态，这个两边都要改
    for(int i=0;i<14;i++)
    {
        joint_position_[i]=joint_init_angle[i];
        joint_position_command_[i]=joint_init_angle[i];
        shared_mem[i]=(float)joint_init_angle[i];
        shared_mem[i+14]=(float)joint_init_angle[i];
    }
    non_realtime_loop_ = nh_.createTimer(update_freq, &YMBOTDHW::update, this);
}

YMBOTDHW::~YMBOTDHW() {
    shmdt(shared_mem);
    std::cout<<"#############################################\n";
    std::cout<<"#########ymbot ros_control loop exit#########\n";
    std::cout<<"#############################################\n";
}

void YMBOTDHW::init() {
    joint_name_={
        "Left_Arm_Joint1","Left_Arm_Joint2","Left_Arm_Joint3","Left_Arm_Joint4","Left_Arm_Joint5","Left_Arm_Joint6","Left_Arm_Joint7",
        "Right_Arm_Joint1","Right_Arm_Joint2","Right_Arm_Joint3","Right_Arm_Joint4","Right_Arm_Joint5","Right_Arm_Joint6","Right_Arm_Joint7",
        "Body_Joint1","Body_Joint2","Body_Joint3","Body_Joint4","Neck_Joint1","Neck_Joint2"
    };
    joint_position_.resize(joint_num,0.0);
    joint_velocity_.resize(joint_num,0.0);
    joint_effort_.resize(joint_num,0.0);
    joint_position_command_.resize(joint_num,0.0);


    for(int i=0;i<joint_num;i++)
    {
        // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        position_joint_interface_.registerHandle(jointPositionHandle);
    }
    // Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);


}

void YMBOTDHW::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void YMBOTDHW::read() {
    // 将关节状态读取到mpc_observation中，共享内存的前十四个是当前关节状态
    // std::cout<<"读取状态"<<"\n";
    for (int i = 0; i < 14; i++) 
    {
        shared_mem[i]= joint_position_[i];
        // std::cout<<shared_mem[i]<<" ";
    }
    // std::cout<<"\n";
}

void YMBOTDHW::write(ros::Duration elapsed_time) {
    //将控制指令发布，共享内存的后十四个是mpc_policy的控制指令
    // std::cout<<"写入控制"<<"\n";
    for (int i = 0; i < 14; ++i) 
    {
        
        joint_position_[i]=shared_mem[i+14];
        // std::cout<<shared_mem[i+14]<<" ";
    }
    // std::cout<<"\n";
}

#endif