#include <ymbot_d_ros_control/ymobot_hardware_interface_real.h>
#include "ymbot_devices_driver/ymbot_joint_eu.h"
#include<thread>
#include <signal.h>
#include <atomic>


// 设定电机组数
int n_motor_group = 3;
int channel = 0;
vector<int> motors_id = {31, 32, 33, 34, 35, 36, 37, 
                         41, 42, 43, 44, 45, 46, 47,
                         11, 12, 13, 14, 21, 22 };//左臂，右臂,上肢

vector<YmbotJointEu> motors(joint_num);
vector<string> joints_name(joint_num);

// 电机通讯异常后的退出函数
void free_canables() {
    for (int i = 0; i < n_motor_group; i++) {
        if (PLANET_SUCCESS != planet_freeDLL(i)) {
            ROS_INFO_STREAM("Motor communication canable" << i << " abnormality......exit");
        }
        else {
            ROS_ERROR_STREAM("Motor communication canable" << i << " closed successfully");
        }
    }
}

std::atomic<bool> g_shutdown_requested(false);
void signalHandler(int signal) {
    if (signal == SIGINT) {
        ROS_INFO("停止！！！！！\n");
        for (size_t j = 0u; j < joint_num; j++) 
        {
            if (motors[j].motor_disabled()) 
            {
                std::cout<<"motor " << motors[j].motor_id << " disabled successfully";
            }
            else 
            {
                std::cout<<"motor " << motors[j].motor_id << " disabled failed";
            }
            this_thread::sleep_for(chrono::milliseconds(500));
            
        }
        free_canables();
    }
}

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

    // can口 通信初始化
    for (int devIndex = 0; devIndex < n_motor_group; devIndex++) {
        if (PLANET_SUCCESS != planet_initDLL(planet_DeviceType_Canable, devIndex, channel, planet_Baudrate_1000)) {
            ROS_ERROR_STREAM("Canable " << devIndex << " communication initialization failed !!!");
            free_canables();
            // return false;
        }
    }
    ROS_INFO_STREAM("Motor communication initialization successfully");

    if (joints_offset_angle.size() != joint_num) {
        ROS_ERROR_STREAM("The number of motors and the number of compensation angles do not match.");
        free_canables();
        // return false;
    }

    // 初始化电机对象，设定id，分组并且给每个电机设定所属的组号，设置角度偏差（与关节零位相关）
    for (int i = 0; i < joint_num; i++) {
        if (motors_id[i] > 10 && motors_id[i] < 30) {
            motors[i].dev_index = 0;
        }
        else if (motors_id[i] > 30 && motors_id[i] < 40) {
            motors[i].dev_index = 1;
        }
        else if (motors_id[i] > 40 && motors_id[i] < 50) {
            motors[i].dev_index = 2;
        }
        else {
            ROS_ERROR_STREAM("There is an id number: "
                             << motors[i].motor_id << " in the motor_id_array that does not match the actual motor.");
            free_canables();
            // return false;
        }
        motors[i].motor_id = motors_id[i];
        motors[i].joint_offset_angle = joints_offset_angle[i];
        motors[i].joint_offset_radian = motors[i].joint_offset_angle / 180.0 * M_PI;
    }

    // 初始化和使能电机
    for (int i = 0; i < joint_num; i++) {
        if (motors[i].motor_initialization_CSP()) {
            // ROS_INFO_STREAM("motor " << motors[i].motor_id << " enabled successfully");
            cout << "motor " << motors[i].motor_id << " enabled successfully" << endl;
            this_thread::sleep_for(chrono::milliseconds(100));
        }
        else {
            cout << "motor " << motors[i].motor_id << " enabled failed" << endl;
            for (size_t j = 0u; j < joint_num; j++) {
                if (motors[j].motor_disabled()) {
                    ROS_INFO_STREAM("motor " << motors[j].motor_id << " disabled successfully");
                }
                else {
                    ROS_ERROR_STREAM("motor " << motors[j].motor_id << " disabled failed");
                }
                this_thread::sleep_for(chrono::milliseconds(500));
            }
            ROS_ERROR_STREAM("Motor communication initialization failed.");
            free_canables();
            // return false;
        }

        // Proactive acquisition of first position to avoid unreasonable values
        if (PLANET_SUCCESS !=
            planet_getPosition(motors[i].dev_index, motors[i].motor_id, &motors[i].present_position)) {
            ROS_ERROR_STREAM("Motor " << motors[i].motor_id << " get position failed");
            free_canables();
            // return false;
        };
        motors[i].present_joint_radian = motors[i].present_position / 180.0 * M_PI - motors[i].joint_offset_radian;
        motors[i].target_position = motors[i].present_position;
        joint_position_[i]=motors[i].present_joint_radian;
        joint_position_command_[i]=motors[i].present_joint_radian;
    }
    for(int i=14;i<20;i++)
    {
        joint_position_[i]=0;
        joint_position_command_[i]=0;
    }
    ROS_INFO_STREAM("All motors initialized successfully");
}

void YMBOTDHW::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void YMBOTDHW::read() {//头不动

    for (int i = 0; i < 14; i++) {
        planet_getPosition(motors[i].dev_index, motors[i].motor_id, &motors[i].present_position);
        joint_position_[i]=motors[i].present_position / 180.0 * M_PI - motors[i].joint_offset_radian;
        shared_mem[i]=joint_position_[i];
    }
}

void YMBOTDHW::write(ros::Duration elapsed_time) {
    // joint_position_=joint_position_command_;
    for (int i = 0; i < 7; i++)
    {
        joint_position_command_[i]=shared_mem[i+14];
        // std::cout<<joint_position_command_[i]<<" ";
        motors[i].target_position=(joint_position_command_[i] + motors[i].joint_offset_radian) / M_PI * 180.0;;
        planet_quick_setTargetPosition(motors[i].dev_index, motors[i].motor_id, motors[i].target_position);
    }
    // std::cout<<"\n";
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ymbot_d_ros_control_real");
    ros::NodeHandle nh;
    signal(SIGINT, signalHandler);
    ros::MultiThreadedSpinner spinner(2); 
    YMBOTDHW ROBOT(nh);
    spinner.spin();
    return 0;
}
