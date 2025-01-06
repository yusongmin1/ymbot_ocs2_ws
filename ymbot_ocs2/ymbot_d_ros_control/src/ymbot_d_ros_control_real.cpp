//不用OCS2的moveit+ros_control插件
#include    <ymbot_d_ros_control/ymobot_hardware_interface_real.h>
#include    "ymbot_devices_driver/ymbot_joint_eu.h"
#include    <thread>
#include    <signal.h>
#include    <atomic>


// 设定电机组数
int n_motor_group = 3;
int channel = 0;
vector<int> motors_id = {  31, 32, 33, 34, 35, 36, 37, 
                           41, 42, 43, 44, 45, 46, 47,
                           11, 12, 13, 14, 21, 22 };//左臂，右臂,躯干

vector<YmbotJointEu> motors(joint_num);
vector<string> joints_name(joint_num);
vector<float> joints_offset_angle(joint_num);

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
    non_realtime_loop_ = nh_.createTimer(update_freq, &YMBOTDHW::update, this);
}

YMBOTDHW::~YMBOTDHW() {
    // shmdt(shared_mem);
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
        // int i = motor_index[k];
        planet_getPosition(motors[i].dev_index, motors[i].motor_id, &motors[i].present_position);
        joint_position_[i]=motors[i].present_position / 180.0 * M_PI - motors[i].joint_offset_radian;
    }
        

}

void YMBOTDHW::write(ros::Duration elapsed_time) {
    joint_position_=joint_position_command_;
    for (int i = 0; i < 14; i++)
    {
        // int i = motor_index[k];
        motors[i].target_position=(joint_position_command_[i] + motors[i].joint_offset_radian) / M_PI * 180.0;;
        planet_quick_setTargetPosition(motors[i].dev_index, motors[i].motor_id, motors[i].target_position);
    }
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
