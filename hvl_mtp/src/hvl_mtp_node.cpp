#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_to_point",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Spin in the background so MoveGroupInterface can receive joint states
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread([&exec]() { exec.spin(); }).detach();

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");  // group name from your SRDF
  arm.setPoseReferenceFrame("world");
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);
  arm.setPlanningTime(5.0);

  // Target point for the end effector, in metres
  arm.setPositionTarget(0.275, 0.125, 0.6, "end_effector_link");  // your end-effector link

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    arm.execute(plan);
    RCLCPP_INFO(node->get_logger(), "Reached target");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed");
  }

  rclcpp::shutdown();
  return 0;
}

