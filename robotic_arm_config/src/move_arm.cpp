#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // 1. Initialize ROS 2
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "move_arm_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // 2. Create the MoveIt Interface
  auto const logger = rclcpp::get_logger("move_arm_node");
  
  // "arm" is the group name defined in Setup Assistant
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // -------------------------
  // ACTION 1: Move to "Home"
  // -------------------------
  RCLCPP_INFO(logger, "Planning to 'home' position...");
  
  // Set named target
  move_group_interface.setNamedTarget("home");
  
  // Plan and Execute
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if(success) {
    RCLCPP_INFO(logger, "Plan Successful! Executing...");
    move_group_interface.execute(my_plan);
  } else {
    RCLCPP_ERROR(logger, "Planning Failed!");
  }

  // -------------------------
  // ACTION 2: Move to Random
  // -------------------------
  RCLCPP_INFO(logger, "Planning to Random position...");
  
  move_group_interface.setRandomTarget();
  
  success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if(success) {
    move_group_interface.execute(my_plan);
  }

  rclcpp::shutdown();
  return 0;
}