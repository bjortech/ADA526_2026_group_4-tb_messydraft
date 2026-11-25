#include <cstdio>
#include <atomic>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
 
// move_to_point.cpp
// Subscribes to geometry_msgs/PointStamped on "target_point" and moves the
// end effector to that point with MoveIt 2 (position-only target).
 
using moveit::planning_interface::MoveGroupInterface;
using geometry_msgs::msg::PointStamped;
 
class MoveToPoint : public rclcpp::Node
{
public:
  MoveToPoint()
  : Node("move_to_point",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // Parameters (declared automatically from overrides, so read with defaults)
    planning_group_ = get_parameter_or<std::string>("planning_group", "arm");
    ee_link_        = get_parameter_or<std::string>("ee_link", "end_effector_link");
    base_frame_     = get_parameter_or<std::string>("base_frame", "world");
 
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
 
    // Own callback group so the (blocking) plan/execute in the callback
    // does not block the callbacks MoveGroupInterface itself needs.
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = cb_group_;
 
    sub_ = create_subscription<PointStamped>(
      "/ball_position", 10,
      std::bind(&MoveToPoint::pointCallback, this, std::placeholders::_1), opts);
  }
 
  // MoveGroupInterface needs shared_from_this(), so it can't be built in the constructor
  void init()
  {
    arm_ = std::make_unique<MoveGroupInterface>(shared_from_this(), planning_group_);
    arm_->setPoseReferenceFrame(base_frame_);
    arm_->setEndEffectorLink(ee_link_);
    arm_->setPlanningTime(5.0);
    arm_->setNumPlanningAttempts(5);
    arm_->setMaxVelocityScalingFactor(0.3);
    arm_->setMaxAccelerationScalingFactor(0.3);
    RCLCPP_INFO(get_logger(), "Ready. Publish PointStamped on /target_point (group '%s', ee '%s')",
                planning_group_.c_str(), ee_link_.c_str());
  }
 
private:
  void pointCallback(const PointStamped::SharedPtr msg)
  {
    if (busy_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Still moving, ignoring new target");
      return;
    }
 
    // Bring the point into the planning frame (empty frame_id = already in base frame)
    PointStamped target = *msg;
    if (!target.header.frame_id.empty() && target.header.frame_id != base_frame_) {
      try {
        target.header.stamp = rclcpp::Time(0);   // use latest available transform
        target = tf_buffer_->transform(target, base_frame_, tf2::durationFromSec(1.0));
      } catch (const tf2::TransformException & e) {
        RCLCPP_ERROR(get_logger(), "TF failed (%s -> %s): %s",
                     msg->header.frame_id.c_str(), base_frame_.c_str(), e.what());
        busy_ = false;
        return;
      }
    }
 
    const auto & p = target.point;
    RCLCPP_INFO(get_logger(), "Target in %s: [%.3f, %.3f, %.3f]",
                base_frame_.c_str(), p.x, p.y, p.z);
 
    arm_->setStartStateToCurrentState();
    arm_->setPositionTarget(p.x, p.y, p.z, ee_link_);
 
    MoveGroupInterface::Plan plan;
    if (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      auto result = arm_->execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
        RCLCPP_INFO(get_logger(), "Reached target");
      else
        RCLCPP_ERROR(get_logger(), "Execution failed (code %d)", result.val);
    } else {
      RCLCPP_ERROR(get_logger(), "Planning failed: point unreachable or no IK solution");
    }
    busy_ = false;
  }
 
  std::string planning_group_, ee_link_, base_frame_;
  std::unique_ptr<MoveGroupInterface> arm_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<PointStamped>::SharedPtr sub_;
  std::atomic<bool> busy_{false};
};
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPoint>();
  node->init();
 
  // Multi-threaded so MoveIt's internal callbacks run while our callback blocks
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
 
  rclcpp::shutdown();
  return 0;
}