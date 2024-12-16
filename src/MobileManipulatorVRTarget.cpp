
#include <string>

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <ros/subscriber.h>
#include <mutex>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include "std_msgs/Float32.h"
#include <atomic>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" 
using namespace ocs2;

namespace
{
  scalar_t targetDisplacementVelocity;
  scalar_t targetRotationVelocity;
  scalar_t comHeight;
  vector_t defaultJointState(12);
  vector_t lastVel_(4);
  vector_t changeLimit_(4);
  std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;
  bool first_pose_received = false;
  geometry_msgs::PoseStamped first_current_pose;
  geometry_msgs::PoseStamped target_pose;
  geometry_msgs::Pose first_pose;

} // namespace


TargetTrajectories goalPoseToTargetTrajectories(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                const SystemObservation& observation) {
  // time trajectory
  const scalar_array_t timeTrajectory{observation.time};
  // state trajectory: 3 + 4 for desired position vector and orientation quaternion
  const vector_t target = (vector_t(7) << position, orientation.coeffs()).finished();
  const vector_array_t stateTrajectory{target};
  // input trajectory
  const vector_array_t inputTrajectory{vector_t::Zero(observation.input.size())};

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

auto observationCallback(const ocs2_msgs::mpc_observation::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(latestObservationMutex_);
  latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
};

auto VRCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(latestObservationMutex_);
  if (latestObservation_.time == 0.0)
  {
    return;
  }
  if (!first_pose_received) 
  {
    first_pose = *msg;
    first_pose_received = true;
    ROS_INFO("First pose received.");
    return;
  } 

    geometry_msgs::Pose relative_pose;//相对位移
    relative_pose.position.x = msg->position.x - first_pose.position.x;
    relative_pose.position.y = msg->position.y - first_pose.position.y;
    relative_pose.position.z = msg->position.z - first_pose.position.z;

    tf2::Quaternion q1(msg->orientation.x, msg->orientation.y,
                       msg->orientation.z, msg->orientation.w);
    tf2::Quaternion q2(first_pose.orientation.x, first_pose.orientation.y,
                       first_pose.orientation.z, first_pose.orientation.w);
    tf2::Quaternion relative_orientation = q1 * q2.inverse();

    // 将 relative_pose 加至 current_pose 上，作为新的目标位姿
    target_pose.pose.position.x =
        first_current_pose.pose.position.x + relative_pose.position.x;
    target_pose.pose.position.y =
        first_current_pose.pose.position.y + relative_pose.position.y;
    target_pose.pose.position.z =
        first_current_pose.pose.position.z + relative_pose.position.z;

    tf2::Quaternion current_orientation(first_current_pose.pose.orientation.x,
                                        first_current_pose.pose.orientation.y,
                                        first_current_pose.pose.orientation.z,
                                        first_current_pose.pose.orientation.w);
    current_orientation = current_orientation * relative_orientation;
    current_orientation.normalize();

    target_pose.pose.orientation.x = current_orientation.x();
    target_pose.pose.orientation.y = current_orientation.y();
    target_pose.pose.orientation.z = current_orientation.z();
    target_pose.pose.orientation.w = current_orientation.w();

    // ROS_INFO("Current pose: Position [%.2f, %.2f, %.2f], Orientation[%.2f, %.2f, %.2f, %.2f]",current_pose.pose.position.x,
    //             current_pose.pose.position.y, current_pose.pose.position.z,
    //             current_pose.pose.orientation.x,
    //             current_pose.pose.orientation.y,
    //             current_pose.pose.orientation.z,
    //             current_pose.pose.orientation.w);
  
  Eigen::Vector3d position_(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
  Eigen::Quaterniond orientation_(
        target_pose.pose.orientation.w, target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z
    );
  const auto trajectories = goalPoseToTargetTrajectories(position_, orientation_,latestObservation_);
  targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
};




int main(int argc, char *argv[])
{
  const std::string robotName = "mobile_manipulator";
  setlocale(LC_ALL,"");
  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
    
  // tf2_ros::Buffer buffer;
  // tf2_ros::TransformListener listener(buffer);
  // std::cout<<"开始启动接受VR信号\n";
  // // geometry_msgs::TransformStamped tfs = buffer.lookupTransform
  // geometry_msgs::TransformStamped  transformStamped = buffer.lookupTransform("Body_Link1", "world", ros::Time(0));
  // first_current_pose.header = transformStamped.header;
  // // geometry_msgs::PoseStamped first_current_pose;
  // first_current_pose.pose.position.x = transformStamped.transform.translation.x;
  // first_current_pose.pose.position.y = transformStamped.transform.translation.y;
  // first_current_pose.pose.position.z = transformStamped.transform.translation.z;
  // first_current_pose.pose.orientation.x = transformStamped.transform.rotation.x;
  // first_current_pose.pose.orientation.y = transformStamped.transform.rotation.y;
  // first_current_pose.pose.orientation.z = transformStamped.transform.rotation.z;
  // first_current_pose.pose.orientation.w = transformStamped.transform.rotation.w;
  first_current_pose.pose.position.x = 0.22806;
  first_current_pose.pose.position.y =0.29645;
  first_current_pose.pose.position.z = 0.28248;
  first_current_pose.pose.orientation.x = 0;
  first_current_pose.pose.orientation.y = 0;
  first_current_pose.pose.orientation.z = 0;
  first_current_pose.pose.orientation.w = 1;
  targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, robotName));

  ::ros::Subscriber observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(
      robotName + "_mpc_observation", 1, observationCallback);

  ::ros::Subscriber vrSub_ = nodeHandle.subscribe<geometry_msgs::Pose>("/vr", 1, VRCallback);

  ::ros::WallRate rosRate(100);
  scalar_t duration = 1.0 / 100.0;

  while (::ros::ok() && ::ros::master::check())
  {
    ::ros::spinOnce();
    rosRate.sleep();
  }

  // Successful exit
  return 0;
}
