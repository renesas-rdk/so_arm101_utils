// ********************************************************************************************************************
// Copyright [2026] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
//
// The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
// and/or its licensors ("Renesas") and subject to statutory and contractual protections.
//
// Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
// display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
// purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
// SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
// NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
// INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
// OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
// be subject to different terms.
// ********************************************************************************************************************

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "control_msgs/action/parallel_gripper_command.hpp"
#include "control_msgs/msg/gripper_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class GripperActionAdapter : public rclcpp::Node
{
public:
  using ParallelGripperCommand = control_msgs::action::ParallelGripperCommand;
  using GoalHandleParallelGripperCommand = rclcpp_action::ServerGoalHandle<ParallelGripperCommand>;

  explicit GripperActionAdapter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gripper_action_adapter", options)
  {
    // Declare parameters with default values
    this->declare_parameter("action_server_name", "gripper_cmd");
    this->declare_parameter("gripper_command_topic", "gripper_command");
    this->declare_parameter(
      "position_controller_topic", "/so_arm101_gripper_position_controller/commands");
    this->declare_parameter("max_gripper_width", 1.0);   // Maximum gripper opening (normalized 0-1)
    this->declare_parameter("execution_duration", 0.1);  // Time to assume goal completion (seconds)

    // Get parameters
    action_server_name_ = this->get_parameter("action_server_name").as_string();
    gripper_command_topic_ = this->get_parameter("gripper_command_topic").as_string();
    position_controller_topic_ = this->get_parameter("position_controller_topic").as_string();
    max_gripper_width_ = this->get_parameter("max_gripper_width").as_double();
    execution_duration_ = this->get_parameter("execution_duration").as_double();

    // Create action server
    action_server_ = rclcpp_action::create_server<ParallelGripperCommand>(
      this, action_server_name_,
      std::bind(
        &GripperActionAdapter::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GripperActionAdapter::handle_cancel, this, std::placeholders::_1),
      std::bind(&GripperActionAdapter::handle_accepted, this, std::placeholders::_1));

    // Create publisher for position commands
    position_command_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(position_controller_topic_, 10);

    // Create subscriber for simple gripper commands
    gripper_command_sub_ = this->create_subscription<control_msgs::msg::GripperCommand>(
      gripper_command_topic_, 10,
      std::bind(&GripperActionAdapter::gripper_command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SO ARM101 Gripper Action Adapter initialized");
    RCLCPP_INFO(this->get_logger(), "Action server: %s", action_server_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper command topic: %s", gripper_command_topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(), "Position controller topic: %s", position_controller_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Execution duration: %.1f seconds", execution_duration_);
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ParallelGripperCommand::Goal> goal)
  {
    (void)uuid;

    RCLCPP_INFO(this->get_logger(), "Received gripper goal request");
    // Extract position from JointState command (assume single position value)
    double goal_position = 0.0;
    double goal_effort = 0.0;

    if (!goal->command.position.empty()) {
      goal_position = goal->command.position[0];
    }
    if (!goal->command.effort.empty()) {
      goal_effort = goal->command.effort[0];
    }

    RCLCPP_INFO(
      this->get_logger(), "Goal position: %.4f, max_effort: %.2f N", goal_position, goal_effort);

    // Validate goal parameters
    if (goal_position < 0.0 || goal_position > max_gripper_width_) {
      RCLCPP_WARN(
        this->get_logger(), "Invalid gripper position: %.4f. Valid range: [0.0, %.4f]",
        goal_position, max_gripper_width_);
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal_effort < 0.0) {
      RCLCPP_WARN(this->get_logger(), "Invalid max_effort: %.2f N. Must be >= 0.0", goal_effort);
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleParallelGripperCommand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel gripper goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleParallelGripperCommand> goal_handle)
  {
    // Execute the goal in a separate thread
    std::thread{std::bind(&GripperActionAdapter::execute, this, std::placeholders::_1), goal_handle}
      .detach();
  }

  void gripper_command_callback(const control_msgs::msg::GripperCommand::SharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(), "Received simple gripper command: position=%.4f, max_effort=%.2f",
      msg->position, msg->max_effort);

    // Validate command parameters
    if (msg->position < 0.0 || msg->position > max_gripper_width_) {
      RCLCPP_WARN(
        this->get_logger(), "Invalid gripper position: %.4f. Valid range: [0.0, %.4f]",
        msg->position, max_gripper_width_);
      return;
    }

    if (msg->max_effort < 0.0) {
      RCLCPP_WARN(
        this->get_logger(), "Invalid max_effort: %.2f N. Must be >= 0.0", msg->max_effort);
      return;
    }

    // Send position command directly (no action feedback needed for simple commands)
    send_position_command(msg->position);
  }

  void send_position_command(double gripper_position)
  {
    // For SO ARM101, gripper position is normalized 0-1 (0=closed, 1=open)
    // Clamp to valid range
    double target_position = std::clamp(gripper_position, 0.0, max_gripper_width_);

    // Create and publish position command (single DOF gripper)
    auto position_cmd = std_msgs::msg::Float64MultiArray();
    position_cmd.data = {target_position};

    RCLCPP_INFO(this->get_logger(), "Sending gripper position command: %.4f", target_position);

    position_command_pub_->publish(position_cmd);
  }

  void execute(const std::shared_ptr<GoalHandleParallelGripperCommand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing gripper goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ParallelGripperCommand::Feedback>();
    auto result = std::make_shared<ParallelGripperCommand::Result>();

    // Extract position from JointState command
    double goal_position = 0.0;
    if (!goal->command.position.empty()) {
      goal_position = goal->command.position[0];
    }

    // Send position command using shared function
    send_position_command(goal_position);

    // Mock execution with configurable duration and periodic feedback
    auto start_time = this->now();
    rclcpp::Rate rate(10);          // 10 Hz feedback rate
    double current_position = 0.0;  // Start from closed position

    while (rclcpp::ok()) {
      // Check if goal was canceled
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        result->state.position = {current_position};
        result->state.effort = {0.0};
        result->stalled = false;
        result->reached_goal = false;
        goal_handle->canceled(result);
        return;
      }

      // Calculate progress (linear interpolation)
      double elapsed_time = (this->now() - start_time).seconds();
      double progress = std::min(elapsed_time / execution_duration_, 1.0);
      current_position = progress * goal_position;

      // Update feedback (using JointState structure)
      feedback->state.position = {current_position};
      feedback->state.effort = {0.0};  // Mock effort feedback

      goal_handle->publish_feedback(feedback);

      // Check if goal is reached
      if (progress >= 1.0) {
        RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
        result->state.position = {goal_position};
        result->state.effort = {0.0};
        result->stalled = false;
        result->reached_goal = true;
        goal_handle->succeed(result);
        return;
      }

      rate.sleep();
    }
  }

  // ROS 2 interfaces
  rclcpp_action::Server<ParallelGripperCommand>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_command_pub_;
  rclcpp::Subscription<control_msgs::msg::GripperCommand>::SharedPtr gripper_command_sub_;

  // Parameters
  std::string action_server_name_;
  std::string gripper_command_topic_;
  std::string position_controller_topic_;
  double max_gripper_width_;
  double execution_duration_;  // Time to assume goal completion
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GripperActionAdapter>();

  RCLCPP_INFO(node->get_logger(), "Starting SO ARM101 Gripper Action Adapter node");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}