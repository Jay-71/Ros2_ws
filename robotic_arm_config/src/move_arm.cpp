#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // 1. Setup Node & Background Spinner
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "automated_move",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("move_arm");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group = MoveGroupInterface(node, "arm");

  // Give the solver time and attempts
  arm_group.setPlanningTime(5.0);
  arm_group.setNumPlanningAttempts(5);

  RCLCPP_INFO(logger, "Waiting for robot state...");
  rclcpp::sleep_for(std::chrono::seconds(2));

  // -------------------------
  // STEP 1: Go Home
  // -------------------------
  RCLCPP_INFO(logger, "Moving to Home...");
  arm_group.setNamedTarget("home");
  arm_group.move();
  arm_group.clearPoseTargets();

  // -------------------------
  // STEP 2: Move to the Golden Coordinate
  // -------------------------
  double target_x = 0.002;
  double target_y = -0.157;
  double target_z = 0.076;

  RCLCPP_INFO(logger, "Moving to Golden Coordinate: X=%.3f, Y=%.3f, Z=%.3f", target_x, target_y, target_z);
  
  // Use setPositionTarget so it ignores orientation and just gets the tip there
  arm_group.setPositionTarget(target_x, target_y, target_z);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if(success) {
    RCLCPP_INFO(logger, "Plan Found! Executing...");
    arm_group.execute(my_plan);
    RCLCPP_INFO(logger, "Arrived successfully!");
  } else {
    RCLCPP_ERROR(logger, "Planning failed even on a known good coordinate. Check IK solver settings.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}