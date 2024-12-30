//控制机器人移动到某个单一关节角度的点，实际机器人，仿真机器人都可以
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "ros/ros.h"
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
#include <sensor_msgs/Joy.h>
#include "std_msgs/String.h"
#include <string.h>
#include <iostream>
#include <vector>
#include "ymbot_devices_driver/ymbot_joint_eu.h"
#include<unistd.h>
using namespace std;



int main(int argc, char** argv) {


    ros::init(argc, argv, "move_to_start_pose_node");
    ros::NodeHandle node_handle;

    moveit::planning_interface::MoveGroupInterface move_group_left_arm("left_arm");
    moveit::planning_interface::MoveGroupInterface move_group_right_arm("right_arm");
        vector<double> joint_group_positions_left={-0.4583,0.3746,0.5241,0.1204,-0.5028,-0.5396,-0.2679};
    vector<double> joint_group_positions_right={0.4583,-0.3746,-0.5241,-0.1204,0.5027,0.5396,0.2679};
    std::map<std::string, double> initial_positions;
    // 获取initial_positions参数
    if (ros::param::get("initial_positions", initial_positions)) {
        std::cout << "Initial Positions: \n";
        int num=0;
        for (const auto& pair : initial_positions) {
            if(num<7)
            {   
                joint_group_positions_left[num]=(double)pair.second;
                ROS_INFO("%s: %f", pair.first.c_str(), pair.second);
            }
            else
            {
                joint_group_positions_right[num-7]=(double)pair.second;
                ROS_INFO("%s: %f", pair.first.c_str(), pair.second);
            }
            num++;
        }
        std::cout << std::endl;
    } 
    ros::AsyncSpinner spinner(2);
    spinner.start();
    bool success_left= false,success_right=false;
    moveit::planning_interface::MoveGroupInterface::Plan plan_left,plan_right;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;



    move_group_left_arm.setMaxVelocityScalingFactor(0.5);
    move_group_left_arm.setMaxAccelerationScalingFactor(0.05);
    move_group_left_arm.setStartStateToCurrentState();
    move_group_left_arm.setJointValueTarget(joint_group_positions_left);

    move_group_right_arm.setMaxVelocityScalingFactor(0.5);
    move_group_right_arm.setMaxAccelerationScalingFactor(0.05);
    move_group_right_arm.setStartStateToCurrentState();
    move_group_right_arm.setJointValueTarget(joint_group_positions_right);

    success_left = (move_group_left_arm.plan(plan_left) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success_left) {
        // Execute the plan
        moveit::core::MoveItErrorCode execution_status = move_group_left_arm.execute(plan_left);
        if (execution_status == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("left_arm Trajectory execution succeeded");
            // Add any subsequent actions here
        }
        else {
            ROS_WARN(" left_arm Trajectory execution failed with error code: %d", execution_status.val);
            return 0;
        }
    }
    else {
        ROS_WARN(" left_arm Planning failed");
        return 0;
    }
    sleep(1);

    success_right = (move_group_right_arm.plan(plan_right) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success_right) {
        // Execute the plan
        moveit::core::MoveItErrorCode execution_status = move_group_right_arm.execute(plan_right);
        if (execution_status == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("right_arm Trajectory execution succeeded");
            // Add any subsequent actions here
        }
        else {
            ROS_WARN(" right_arm Trajectory execution failed with error code: %d", execution_status.val);
            return 0;
        }
    }
    else {
        ROS_WARN(" right_arm Planning failed");
        return 0;
    }
    return 0;
}