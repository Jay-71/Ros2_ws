#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "position_monitor_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("monitor");

  // Spin in background to hear joint_states
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group = MoveGroupInterface(node, "arm");

  RCLCPP_INFO(logger, "-------------------------------------------");
  RCLCPP_INFO(logger, "POSITION MONITOR STARTED");
  RCLCPP_INFO(logger, "Waiting 2 seconds for robot to connect...");
  RCLCPP_INFO(logger, "-------------------------------------------");

  // Wait for the first joint_states to arrive so it doesn't crash
  rclcpp::sleep_for(std::chrono::seconds(2));

  while (rclcpp::ok()) {
    try {
      geometry_msgs::msg::PoseStamped current_pose = arm_group.getCurrentPose();
      
      // Using standard RCLCPP_INFO so the launch file actually displays it
      RCLCPP_INFO(logger, "LIVE POSE -> X: %.3f | Y: %.3f | Z: %.3f", 
             current_pose.pose.position.x, 
             current_pose.pose.position.y, 
             current_pose.pose.position.z);

    } catch (...) {
      // Ignore errors while waiting for tf tree to build
    }
    
    // Sleep for 0.5 seconds (2Hz) to prevent terminal spam
    rclcpp::sleep_for(std::chrono::milliseconds(500)); 
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}