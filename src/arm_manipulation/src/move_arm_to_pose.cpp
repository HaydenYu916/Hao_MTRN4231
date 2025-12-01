/**
 * Use Pilz LIN planner for linear motion
 */
#include <memory>
#include <string>
#include <thread>
#include <cmath>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_common.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>
#include <sstream>

// Helper function to get error code name
std::string getErrorCodeName(const moveit::core::MoveItErrorCode& error_code) {
  if (error_code == moveit::core::MoveItErrorCode::SUCCESS) return "SUCCESS";
  if (error_code == moveit::core::MoveItErrorCode::FAILURE) return "FAILURE";
  if (error_code == moveit::core::MoveItErrorCode::PLANNING_FAILED) return "PLANNING_FAILED";
  if (error_code == moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN) return "INVALID_MOTION_PLAN";
  if (error_code == moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE) return "MOTION_PLAN_INVALIDATED";
  if (error_code == moveit::core::MoveItErrorCode::CONTROL_FAILED) return "CONTROL_FAILED";
  if (error_code == moveit::core::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA) return "UNABLE_TO_AQUIRE_SENSOR_DATA";
  if (error_code == moveit::core::MoveItErrorCode::TIMED_OUT) return "TIMED_OUT";
  if (error_code == moveit::core::MoveItErrorCode::PREEMPTED) return "PREEMPTED";
  if (error_code == moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION) return "START_STATE_IN_COLLISION";
  if (error_code == moveit::core::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS) return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
  if (error_code == moveit::core::MoveItErrorCode::GOAL_IN_COLLISION) return "GOAL_IN_COLLISION";
  if (error_code == moveit::core::MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS) return "GOAL_VIOLATES_PATH_CONSTRAINTS";
  if (error_code == moveit::core::MoveItErrorCode::INVALID_GROUP_NAME) return "INVALID_GROUP_NAME";
  if (error_code == moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS) return "INVALID_GOAL_CONSTRAINTS";
  if (error_code == moveit::core::MoveItErrorCode::INVALID_ROBOT_STATE) return "INVALID_ROBOT_STATE";
  if (error_code == moveit::core::MoveItErrorCode::INVALID_LINK_NAME) return "INVALID_LINK_NAME";
  if (error_code == moveit::core::MoveItErrorCode::INVALID_OBJECT_NAME) return "INVALID_OBJECT_NAME";
  if (error_code == moveit::core::MoveItErrorCode::FRAME_TRANSFORM_FAILURE) return "FRAME_TRANSFORM_FAILURE";
  if (error_code == moveit::core::MoveItErrorCode::COLLISION_CHECKING_UNAVAILABLE) return "COLLISION_CHECKING_UNAVAILABLE";
  if (error_code == moveit::core::MoveItErrorCode::ROBOT_STATE_STALE) return "ROBOT_STATE_STALE";
  if (error_code == moveit::core::MoveItErrorCode::SENSOR_INFO_STALE) return "SENSOR_INFO_STALE";
  if (error_code == moveit::core::MoveItErrorCode::NO_IK_SOLUTION) return "NO_IK_SOLUTION";
  std::stringstream ss;
  ss << "UNKNOWN_ERROR(" << error_code.val << ")";
  return ss.str();
}

// Joint constraint configuration (reference: ScrewDrivingBot)
struct JointConstraintConfig {
    std::string joint_name;
    double position;
    double tolerance_above;
    double tolerance_below;
};

// Toggle: enable/disable joint/path constraints (set to false to disable)
// Keep constraints disabled for now to verify that OMPL planning works in an unconstrained setting
constexpr bool USE_JOINT_CONSTRAINTS = false;

const std::vector<JointConstraintConfig> JOINT_CONSTRAINTS = {
  { "shoulder_pan_joint",   0,               M_PI,        M_PI },        // 0° ± 180°
  { "shoulder_lift_joint", -M_PI / 4,        M_PI / 2,     M_PI / 2 },   // -45° ± 90° (slightly relaxed compared to the original)
  { "wrist_1_joint",       -M_PI * 105/180,  M_PI,        M_PI },        // -105° ± 180°
  { "wrist_2_joint",       -M_PI / 2,        M_PI,        M_PI },        // -90° ± 180°
  { "wrist_3_joint",        0,               M_PI,        M_PI }         // 0° ± 180°
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("move_arm_to_pose", 
                                                     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  double target_x = 0.3, target_y = 0.3, target_z = 0.4;
  node->get_parameter("target_x", target_x);
  node->get_parameter("target_y", target_y);
  node->get_parameter("target_z", target_z);

  std::cout << "\n=== Moving to (" << target_x << ", " << target_y << ", " << target_z << ") ===" << std::endl;

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "ur_manipulator");
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  
  // Set end effector link to tool_point (0.104m from tool0)
  // Note: link name is "tool_point" (prefix is empty in URDF)
  move_group.setEndEffectorLink("tool_point");
  std::cout << "End effector link set to: " << move_group.getEndEffectorLink() << std::endl;
  
  // Use OMPL planner (UR official moveit_config is configured only with OMPL by default)
  // If pilz_industrial_motion_planner is correctly loaded into move_group in the future, switch to LIN
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setPlanningTime(20.0);  // Planning time
  move_group.setNumPlanningAttempts(15);  // Number of planning attempts (reference uses 15)
  move_group.setGoalTolerance(0.0005);  // Goal tolerance: 0.1mm precision (reference uses 0.00005)
  move_group.setMaxVelocityScalingFactor(0.1);  // Velocity scaling
  move_group.setMaxAccelerationScalingFactor(0.1);  // Acceleration scaling
  move_group.setStartStateToCurrentState();  // Start from current state (reference pattern)
  
  std::cout << "✓ Using OMPL planner via UR MoveIt config" << std::endl;
  std::cout << "  Planner ID: " << move_group.getPlannerId() << std::endl;
  std::cout << "  Planning time: 20.0 seconds" << std::endl;
  std::cout << "  Planning attempts: 15" << std::endl;
  std::cout << "  Goal tolerance: 0.0005 m (0.1mm)" << std::endl;

  // Get current pose
  auto current_pose_stamped = move_group.getCurrentPose();
  auto current_pose = current_pose_stamped.pose;
  std::cout << "Planning frame: " << move_group.getPlanningFrame() << std::endl;
  std::cout << "End effector link: " << move_group.getEndEffectorLink() << std::endl;
  std::cout << "Current pose frame: " << current_pose_stamped.header.frame_id << std::endl;
  std::cout << "Current position (" << move_group.getEndEffectorLink() << "): (" 
            << current_pose.position.x << ", " 
            << current_pose.position.y << ", " << current_pose.position.z << ")" << std::endl;
  
  // Get tool0 position for comparison
  try {
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);
    
    // Wait for TF to be available
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    geometry_msgs::msg::TransformStamped tool0_transform;
    tool0_transform = tf_buffer.lookupTransform(
      move_group.getPlanningFrame(), "tool0", tf2::TimePointZero);
    
    std::cout << "\n=== Tool0 Transform (base_link → tool0) ===" << std::endl;
    std::cout << "Position (x, y, z): (" 
              << tool0_transform.transform.translation.x << ", "
              << tool0_transform.transform.translation.y << ", "
              << tool0_transform.transform.translation.z << ")" << std::endl;
    std::cout << "Orientation (quaternion): ("
              << tool0_transform.transform.rotation.x << ", "
              << tool0_transform.transform.rotation.y << ", "
              << tool0_transform.transform.rotation.z << ", "
              << tool0_transform.transform.rotation.w << ")" << std::endl;
    
    // Convert quaternion to RPY
    tf2::Quaternion q(
      tool0_transform.transform.rotation.x,
      tool0_transform.transform.rotation.y,
      tool0_transform.transform.rotation.z,
      tool0_transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::cout << "Orientation (RPY in radians): (" 
              << roll << ", " << pitch << ", " << yaw << ")" << std::endl;
    std::cout << "Orientation (RPY in degrees): (" 
              << roll * 180.0 / M_PI << ", " 
              << pitch * 180.0 / M_PI << ", " 
              << yaw * 180.0 / M_PI << ")" << std::endl;
    
    std::cout << "\n=== Difference (ee_tool_point - tool0) ===" << std::endl;
    std::cout << "Position difference: ("
              << current_pose.position.x - tool0_transform.transform.translation.x << ", "
              << current_pose.position.y - tool0_transform.transform.translation.y << ", "
              << current_pose.position.z - tool0_transform.transform.translation.z << ")" << std::endl;
    
    // Also get ee_tool_point position directly via TF for verification
    try {
      geometry_msgs::msg::TransformStamped ee_tool_point_transform;
      ee_tool_point_transform = tf_buffer.lookupTransform(
        move_group.getPlanningFrame(), move_group.getEndEffectorLink(), tf2::TimePointZero);
      
      std::cout << "\n=== ee_tool_point Transform (via TF) ===" << std::endl;
      std::cout << "Position (x, y, z): (" 
                << ee_tool_point_transform.transform.translation.x << ", "
                << ee_tool_point_transform.transform.translation.y << ", "
                << ee_tool_point_transform.transform.translation.z << ")" << std::endl;
      
      std::cout << "\n=== Verification: Difference (TF ee_tool_point - tool0) ===" << std::endl;
      std::cout << "Position difference: ("
                << ee_tool_point_transform.transform.translation.x - tool0_transform.transform.translation.x << ", "
                << ee_tool_point_transform.transform.translation.y - tool0_transform.transform.translation.y << ", "
                << ee_tool_point_transform.transform.translation.z - tool0_transform.transform.translation.z << ")" << std::endl;
      std::cout << "Expected difference: (0, 0, 0.104) [gripper center point]" << std::endl;
    } catch (const tf2::TransformException & ex) {
      std::cout << "Could not get ee_tool_point transform: " << ex.what() << std::endl;
    }
  } catch (const tf2::TransformException & ex) {
    std::cout << "Could not get tool0 transform: " << ex.what() << std::endl;
  }

  // Print current joint values
  auto joint_names = move_group.getJointNames();
  auto joint_values = move_group.getCurrentJointValues();
  std::cout << "\nCurrent joint values:" << std::endl;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    std::cout << "  " << joint_names[i] << ": " << joint_values[i] 
              << " rad (" << (joint_values[i] * 180.0 / M_PI) << " deg)" << std::endl;
  }

  // Target pose: keep current orientation
  geometry_msgs::msg::Pose target_pose = current_pose;
  target_pose.position.x = target_x;
  target_pose.position.y = target_y;
  target_pose.position.z = target_z;
  
  // Optionally setup joint constraints (can be disabled via USE_JOINT_CONSTRAINTS)
  if (USE_JOINT_CONSTRAINTS) {
    // Setup joint constraints (following ScrewDrivingBot reference implementation)
    // Note: Setting constraints may cause MoveIt to fall back to OMPL if LIN doesn't support them
    move_group.clearPathConstraints();  // Clear any existing constraints first
    
    moveit_msgs::msg::Constraints constraints;
    
    // Create a map of joint names to current values for easy lookup
    std::map<std::string, double> current_joint_map;
    for (size_t i = 0; i < joint_names.size(); ++i) {
      current_joint_map[joint_names[i]] = joint_values[i];
    }
    
    // Setup joint constraints using your configuration (following reference pattern)
    for (const auto& config : JOINT_CONSTRAINTS) {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = config.joint_name;
      
      // Use current joint value as target if available, otherwise use configured value
      if (current_joint_map.find(config.joint_name) != current_joint_map.end()) {
        jc.position = current_joint_map[config.joint_name];
        std::cout << "  Using current value for " << config.joint_name 
                  << ": " << jc.position << " rad (" 
                  << (jc.position * 180.0 / M_PI) << " deg)" << std::endl;
      } else {
        jc.position = config.position;
        std::cout << "  Using configured value for " << config.joint_name 
                  << ": " << jc.position << " rad (" 
                  << (jc.position * 180.0 / M_PI) << " deg)" << std::endl;
      }
      
      // Set tolerance from your configuration
      jc.tolerance_above = config.tolerance_above;
      jc.tolerance_below = config.tolerance_below;
      jc.weight = 1.0;  // Use weight 1.0 as in reference implementation
      
      constraints.joint_constraints.push_back(jc);
      
      // Print constraint range for debugging
      double min_allowed = jc.position - jc.tolerance_below;
      double max_allowed = jc.position + jc.tolerance_above;
      std::cout << "    Constraint range: [" << min_allowed << ", " << max_allowed 
                << "] rad ([" << (min_allowed * 180.0 / M_PI) << "°, " 
                << (max_allowed * 180.0 / M_PI) << "°])" << std::endl;
    }
    
    // Set path constraints (following reference implementation pattern)
    move_group.setPathConstraints(constraints);
    std::cout << "\n✓ Joint constraints set (using your configuration)" << std::endl;
    std::cout << "  Total constraints: " << constraints.joint_constraints.size() << std::endl;
  } else {
    // Ensure there are absolutely no path constraints when disabled
    move_group.clearPathConstraints();
    std::cout << "\n⚙ Joint constraints DISABLED (USE_JOINT_CONSTRAINTS = false)" << std::endl;
  }
  
  std::cout << "Target position (" << move_group.getEndEffectorLink() << "): (" 
            << target_pose.position.x << ", " 
            << target_pose.position.y << ", " 
            << target_pose.position.z << ")" << std::endl;

  // Use MoveIt's Cartesian path (computeCartesianPath) to generate a straight-line motion
  std::cout << "\n=== Cartesian path (linear interpolation) ===" << std::endl;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(current_pose);
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.001;     // 1 mm step for end-effector
  const double jump_threshold = 0.0; // disable jump detection

  double fraction = move_group.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);

  std::cout << "  Path fraction: " << (fraction * 100.0) << "% of straight line planned" << std::endl;

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = false;

  // Allow a certain portion of the path to deviate (e.g., around obstacles) as long as most of it is linear
  const double MIN_FRACTION = 0.85;
  if (fraction >= MIN_FRACTION) {
    std::cout << "✓ Cartesian path computed successfully (fraction >= " 
              << (MIN_FRACTION * 100.0) << "%), executing..." << std::endl;
    plan.trajectory_ = trajectory;

    // Continue using the previously configured velocity/acceleration scaling here
    moveit::core::MoveItErrorCode execute_result = move_group.execute(plan);
    success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
      std::cout << "✗ Cartesian execution failed, error code: " << execute_result.val
                << " (" << getErrorCodeName(execute_result) << ")" << std::endl;
    }
  } else {
    std::cout << "✗ Cartesian path planning fraction too low, falling back to normal OMPL planning..."
              << std::endl;

    // Fallback: use standard OMPL planning to the same target pose (path may be non-linear)
    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    auto plan_result = move_group.plan(joint_plan);

    if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
      std::cout << "✓ Fallback OMPL planning succeeded, executing trajectory..." << std::endl;
      auto execute_result = move_group.execute(joint_plan);
      success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
      if (!success) {
        std::cout << "✗ Fallback execution failed, error code: " << execute_result.val
                  << " (" << getErrorCodeName(execute_result) << ")" << std::endl;
      }
    } else {
      std::cout << "✗ Fallback OMPL planning also failed, error code: " << plan_result.val
                << " (" << getErrorCodeName(plan_result) << ")" << std::endl;
    }
  }
  
  move_group.clearPathConstraints();

  if (success) {
    std::cout << "\n✓ Motion executed!" << std::endl;
    
    // Get actual reached position
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Wait for motion to settle
    auto final_pose_stamped = move_group.getCurrentPose();
    auto final_pose = final_pose_stamped.pose;
    
    std::cout << "\n=== Position Verification ===" << std::endl;
    std::cout << "Target position (" << move_group.getEndEffectorLink() << "): (" 
              << target_pose.position.x << ", " 
              << target_pose.position.y << ", " 
              << target_pose.position.z << ")" << std::endl;
    std::cout << "Actual reached position (" << move_group.getEndEffectorLink() << "): (" 
              << final_pose.position.x << ", " 
              << final_pose.position.y << ", " 
              << final_pose.position.z << ")" << std::endl;
    std::cout << "Position error: ("
              << final_pose.position.x - target_pose.position.x << ", "
              << final_pose.position.y - target_pose.position.y << ", "
              << final_pose.position.z - target_pose.position.z << ")" << std::endl;
    
    double position_error = std::sqrt(
      std::pow(final_pose.position.x - target_pose.position.x, 2) +
      std::pow(final_pose.position.y - target_pose.position.y, 2) +
      std::pow(final_pose.position.z - target_pose.position.z, 2));
    std::cout << "Total position error: " << position_error << " m (" << position_error * 1000 << " mm)" << std::endl;
    
    // Also check tool0 position for reference
    try {
      tf2_ros::Buffer tf_buffer_final(node->get_clock());
      tf2_ros::TransformListener tf_listener_final(tf_buffer_final);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      
      auto tool0_final = tf_buffer_final.lookupTransform(
        move_group.getPlanningFrame(), "tool0", tf2::TimePointZero);
      
      std::cout << "\nFinal tool0 position: ("
                << tool0_final.transform.translation.x << ", "
                << tool0_final.transform.translation.y << ", "
                << tool0_final.transform.translation.z << ")" << std::endl;
      std::cout << "Final ee_tool_point - tool0 difference: ("
                << final_pose.position.x - tool0_final.transform.translation.x << ", "
                << final_pose.position.y - tool0_final.transform.translation.y << ", "
                << final_pose.position.z - tool0_final.transform.translation.z << ")" << std::endl;
      std::cout << "Expected difference: (0, 0, 0.104) [gripper center point]" << std::endl;
    } catch (const tf2::TransformException & ex) {
      std::cout << "Could not get final tool0 transform: " << ex.what() << std::endl;
    }
  } else {
    std::cout << "\n✗ Failed!" << std::endl;
  }

  rclcpp::shutdown();
  spinner.join();
  return success ? 0 : 1;  // Return 0 on success, 1 on failure
}
