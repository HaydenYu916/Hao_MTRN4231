#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

auto generatePoseMsg(float x, float y, float z, float qx, float qy, float qz, float qw) {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    return msg;
}

namespace rvt = rviz_visual_tools;

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("move_arm_to_pose", 
                                                     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Declare/receive target pose parameters (defaults: 0.3, 0.3, 0.4)
  double target_x = 0.3, target_y = 0.3, target_z = 0.4;
  (void)node->get_parameter("target_x", target_x);
  (void)node->get_parameter("target_y", target_y);
  (void)node->get_parameter("target_z", target_z);

  std::cout << "\n========================================" << std::endl;
  std::cout << "Moving arm to target pose" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "Target position: x=" << target_x << ", y=" << target_y << ", z=" << target_z << std::endl;
  std::cout << "========================================\n" << std::endl;

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  move_group_interface.setPlanningTime(10.0);

  std::string frame_id = move_group_interface.getPlanningFrame();

  // Display available joints
  std::cout << "Available joints:" << std::endl;
  auto jointNames = move_group_interface.getJointNames();
  for (std::string i: jointNames) {
    std::cout << "  - " << i << std::endl;
  }
  std::cout << std::endl;

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move_arm_to_pose");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  
  // Create visualisation tool to use with rviz
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{ 
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, 
      move_group_interface.getRobotModel() };

  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Choice of planner
  move_group_interface.setPlannerId("TRRTkConfigDefault");

  // Make sure we're in the same frame as used for constraints
  move_group_interface.setPoseReferenceFrame(move_group_interface.getPlanningFrame());
  
  // Ensure the start state is exactly what the robot thinks it is
  move_group_interface.setStartStateToCurrentState();
  
  // Slow down to avoid edge cases during time parameterization
  move_group_interface.setMaxVelocityScalingFactor(0.5);
  move_group_interface.setMaxAccelerationScalingFactor(0.5);

  // Set target pose (facing down: 0,1,0,0)
  auto target_pose = generatePoseMsg(target_x, target_y, target_z, 0.0, 1.0, 0.0, 0.0);
  move_group_interface.setPoseTarget(target_pose);

  std::cout << "Starting path planning..." << std::endl;

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan planMessage;
  auto success = static_cast<bool>(move_group_interface.plan(planMessage));
    
  // Execute the plan
  if (success) {
    std::cout << "✓ Planning successful! Starting execution..." << std::endl;
    auto exec_success = move_group_interface.execute(planMessage);
    if (exec_success) {
      std::cout << "\n✓✓✓ Movement completed successfully!" << std::endl;
    } else {
      RCLCPP_ERROR(logger, "Execution failed!");
    }
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // AFTER execution (or after a failed attempt), always clear:
  move_group_interface.clearPathConstraints();
  move_group_interface.setStartStateToCurrentState();

  moveit_visual_tools.trigger();

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}

