#include "ymbot_devices_driver/ymbot_joint_eu.h"
#include <vector>
#include<iostream>
#include<cmath>
using namespace std;
#define joint_num 20
// 设定电机组数
int n_motor_group = 3;
int channel = 0;
vector<int> motors_id = 
                    {31, 32, 33, 34, 35, 36, 37, 
                      41, 42, 43, 44, 45, 46, 47,
                      11, 12, 13, 14, 21, 22 };//左臂，右臂,上肢

vector<YmbotJointEu> motors(joint_num);
vector<string> joints_name(joint_num);
vector<float> joints_offset_angle(joint_num);

// 电机通讯异常后的退出函数
void free_canables() {
    for (int i = 0; i < n_motor_group; i++) {
        if (PLANET_SUCCESS != planet_freeDLL(i)) {
            std::cout<<"Motor communication canable" << i << " abnormality......exit";
        }
        else {
            std::cout<<"Motor communication canable" << i << " closed successfully";
        }
    }
}

int main(int argc, char *argv[])
{
    // 初始化电机对象，设定id，分组并且给每个电机设定所属的组号，设置角度偏差（与关节零位相关）
    for (int devIndex = 0; devIndex < 3; devIndex++) {
    if (PLANET_SUCCESS != planet_initDLL(planet_DeviceType_Canable, devIndex,
                                         channel, planet_Baudrate_1000)) {
      cout << "Canable " << devIndex
           << " communication initialization failed !!!" << endl;
      return 0;
    }
  }
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
            std::cout<<"There is an id number: "
                             << motors[i].motor_id << " in the motor_id_array that does not match the actual motor.";
            free_canables();
            // return false;
        }
        motors[i].motor_id = motors_id[i];
        motors[i].joint_offset_angle = joints_offset_angle[i];
        motors[i].joint_offset_radian = motors[i].joint_offset_angle / 180.0 * M_PI;
    }
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
    return 0;
}
