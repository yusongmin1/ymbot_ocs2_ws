#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "ymbot_devices_driver/ymbot_joint_eu.h"

int joint_control_command[14] = {0}, exit_flag = 0;
int keyboard_flag = 0;
bool motors_are_running_flag = false;
double radians_will_publish_to_ros[14] = {0.0};

char keycode_positive[14] = {'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', 'x', 'v', 'n', '1'};
char keycode_negative[14] = {'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', 'z', 'c', 'b', 'm', '2'};
std::atomic<bool> keyboard_running(true);
std::atomic<char> key(0);


// int id_array[] = {21, 22};

// int id_array[] = {31, 32, 33, 34, 35, 36, 37};

int id_array[] = {41, 42, 43, 44, 45, 46, 47};

// int id_array[] = {11, 12, 13, 14, 21, 22};

// int id_array[] = {65};


// int id_array[] = {35};

using namespace std;

int MotorFunction() {
    /**************************************motors****************************************/
    int devIndex = 2;
    int channel = 0;
    int n_motors = sizeof(id_array) / sizeof(id_array[0]);
    vector<YmbotJointEu> motor(n_motors);
    vector<float> joints_offset_angle(n_motors);
    float temp_var1, temp_var2;

    // if (PLANET_SUCCESS != planet_initDLL(planet_DeviceType_USBCAN2, devIndex, channel, planet_Baudrate_1000)) {
    //     cout << "Motor communication initialization failed !!!" << endl;
    //     return 0;
    // }

    if (PLANET_SUCCESS != planet_initDLL(planet_DeviceType_Canable, devIndex, channel, planet_Baudrate_1000)) {
        cout << "Motor communication initialization failed !!!" << endl;
        return 0;
    }

    cout << n_motors << endl;
    for (int i = 0; i < n_motors; i++) {
        motor[i].dev_index = devIndex;
        motor[i].motor_id = id_array[i];
        // motor[i].joint_offset_angle = joints_offset_angle[i];
        // motor[i].joint_offset_radian = motor[i].joint_offset_angle / 180.0 * M_PI;
    }

    cout << endl;
    // for(int i=0; i<sizeof(id_array)/sizeof(id_array[0]); i++){
    //     cout << "Motor " <<  motor[i].motor_id << "position: " << getPosition(motor[i].motor_id) << endl;
    // }

    cout << "****************Motor initialization********************" << endl;
    for (int i = 0; i < n_motors; i++) {
        if (motor[i].motor_initialization_CSP()) {
            cout << "motor " << motor[i].motor_id << " enabled successfully" << endl;
            this_thread::sleep_for(chrono::milliseconds(500));
        }
        else {
            cout << "motor " << motor[i].motor_id << " enabled failed" << endl;
            for (size_t j = 0u; j < n_motors; j++) {
                if (motor[j].motor_disabled()) {
                    cout << "motor " << motor[i].motor_id << " disabled successfully" << endl;
                }
                else {
                    cout << "motor " << motor[i].motor_id << " disabled failed" << endl;
                }
                this_thread::sleep_for(chrono::milliseconds(500));
            }
            planet_freeDLL(devIndex);
            return 0;
        }

        // Proactive acquisition of first position to avoid unreasonable values
        planet_getPosition(motor[i].dev_index, motor[i].motor_id, &motor[i].present_position);
        motor[i].present_joint_radian = motor[i].present_position / 180.0 * M_PI - motor[i].joint_offset_radian;
    }
    cout << "All motors initialized successfully" << endl;

    /**************************************keyboard motors****************************************/
    while (exit_flag == 0) {
        cout << "position is : ";
        for (int i = 0; i < sizeof(id_array) / sizeof(id_array[0]); i++) {
            if (joint_control_command[i] == 1) {
                planet_getPosition(motor[i].dev_index, motor[i].motor_id, &motor[i].present_position);
                motor[i].target_position = motor[i].present_position + 0.5;
                planet_quick_setTargetPosition(motor[i].dev_index, motor[i].motor_id, motor[i].target_position);
                motor[i].record_position = motor[i].target_position;
            }
            else if (joint_control_command[i] == -1) {
                planet_getPosition(motor[i].dev_index, motor[i].motor_id, &motor[i].present_position);
                motor[i].target_position = motor[i].present_position - 0.5;
                planet_quick_setTargetPosition(motor[i].dev_index, motor[i].motor_id, motor[i].target_position);
                motor[i].record_position = motor[i].target_position;
            }
            else {
                motor[i].target_position = motor[i].record_position;
                planet_quick_setTargetPosition(motor[i].dev_index, motor[i].motor_id, motor[i].target_position);
                motor[i].record_position = motor[i].target_position;
            }

            planet_getPosition(motor[i].dev_index, motor[i].motor_id, &motor[i].present_position);
            cout << motor[i].present_position << "\t";
        }
        cout << endl;
        this_thread::sleep_for(chrono::milliseconds(20));
    }

    this_thread::sleep_for(chrono::milliseconds(500));
    for (int j = 0; j < n_motors; j++) {
        motor[j].motor_disabled();
        this_thread::sleep_for(chrono::seconds(1));
    }
    planet_freeDLL(devIndex);
    return 0;
}

// 获取键盘键值线程函数
void KeyboardFunction() {
    struct termios original;
    struct termios modified;

    // 禁用标准输入的行缓冲和回显
    tcgetattr(STDIN_FILENO, &original);
    modified = original;
    modified.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &modified);

    while (keyboard_running) {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            // 将获取到的键值存储在共享变量中
            key.store(c);

            std::cout << c << std::endl;
            if (c == '`') {
                exit_flag = 1;
            }
            else {
                for (int i = 0; i < 14; i++) {
                    if (c == keycode_positive[i]) {
                        joint_control_command[i] = 1;
                    }
                    else if (c == keycode_negative[i]) {
                        joint_control_command[i] = -1;
                    }
                    else {
                        joint_control_command[i] = 0;
                    }
                }
            }
        }
    }
    // 恢复原始终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &original);
}

int main(int argc, char** argv) {
    thread t1(KeyboardFunction);
    t1.detach();
    // sleep(5);

    MotorFunction();

    return 0;
}