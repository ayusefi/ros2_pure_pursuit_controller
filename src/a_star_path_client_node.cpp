#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <a_star_interfaces/action/a_star_planner.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <cmath>

class AStarPathClient : public rclcpp::Node {
public:
  using AStarPlanner = a_star_interfaces::action::AStarPlanner;
  using GoalHandleAStarPlanner = rclcpp_action::ClientGoalHandle<AStarPlanner>;

  AStarPathClient() : Node("a_star_path_client"), has_pose_(false), has_goal_(false) {
    // Action client for A* planner
    a_star_client_ = rclcpp_action::create_client<AStarPlanner>(this, "a_star_planner");
    
    // Publisher for the planned path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);
    
    // Subscriber for robot localized pose from AMCL Lite (start position)
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/amcl_lite_pose", 10,
      std::bind(&AStarPathClient::pose_callback, this, std::placeholders::_1));
    
    // Subscriber for RViz goal pose
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&AStarPathClient::goal_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "A* Path Client started");
    RCLCPP_INFO(this->get_logger(), "Waiting for AMCL Lite pose and goal pose from RViz...");
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
    has_pose_ = true;
    
    if (!has_goal_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Robot localized position: (%.2f, %.2f) - Waiting for goal pose from RViz...", 
        current_pose_.pose.position.x, current_pose_.pose.position.y);
    }
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Check if this is a significantly different goal to avoid spam
    if (has_goal_) {
      double dx = msg->pose.position.x - goal_pose_.pose.position.x;
      double dy = msg->pose.position.y - goal_pose_.pose.position.y;
      double distance = sqrt(dx*dx + dy*dy);
      if (distance < 0.1) {  // Less than 10cm difference, ignore
        return;
      }
    }
    
    goal_pose_ = *msg;
    has_goal_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Received NEW goal pose: (%.2f, %.2f)", 
                goal_pose_.pose.position.x, goal_pose_.pose.position.y);
    
    // Request path when we have both start and goal
    if (has_pose_ && has_goal_) {
      request_path();
    }
  }

  void request_path() {
    if (!a_star_client_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_ERROR(this->get_logger(), "A* action server not available after waiting 3 seconds");
      return;
    }

    auto goal_msg = AStarPlanner::Goal();
    
    // Set start position from current robot pose
    goal_msg.start_pose = current_pose_;
    goal_msg.start_pose.header.frame_id = "map";  // Ensure map frame
    goal_msg.start_pose.header.stamp = this->now();
    
    // Set goal position from RViz
    goal_msg.goal_pose = goal_pose_;
    goal_msg.goal_pose.header.frame_id = "map";   // Ensure map frame
    goal_msg.goal_pose.header.stamp = this->now();

    auto send_goal_options = rclcpp_action::Client<AStarPlanner>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleAStarPlanner::SharedPtr & goal_handle) {
        this->goal_response_callback(goal_handle);
      };
    send_goal_options.feedback_callback =
      [this](GoalHandleAStarPlanner::SharedPtr goal_handle, 
              const std::shared_ptr<const AStarPlanner::Feedback> feedback) {
        this->feedback_callback(goal_handle, feedback);
      };
    send_goal_options.result_callback =
      [this](const GoalHandleAStarPlanner::WrappedResult & result) {
        this->result_callback(result);
      };

    RCLCPP_INFO(this->get_logger(), "Sending goal to A* planner");
    RCLCPP_INFO(this->get_logger(), "Start: (%.2f, %.2f) -> Goal: (%.2f, %.2f) [map frame]", 
                current_pose_.pose.position.x, current_pose_.pose.position.y,
                goal_pose_.pose.position.x, goal_pose_.pose.position.y);
    
    a_star_client_->async_send_goal(goal_msg, send_goal_options);
    
    // Don't reset goal flag here - wait for result
  }

  void goal_response_callback(const GoalHandleAStarPlanner::SharedPtr & goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleAStarPlanner::SharedPtr,
    const std::shared_ptr<const AStarPlanner::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Nodes explored: %d", feedback->number_of_nodes_explored);
  }

  void result_callback(const GoalHandleAStarPlanner::WrappedResult & result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Path planning succeeded!");
        publish_path(result.result->path);
        // Reset goal flag after successful planning
        has_goal_ = false;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Path planning was aborted");
        // Reset goal flag to allow retry
        has_goal_ = false;
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Path planning was canceled");
        // Reset goal flag to allow retry
        has_goal_ = false;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        has_goal_ = false;
        break;
    }
  }

  void publish_path(const nav_msgs::msg::Path& path) {
    RCLCPP_INFO(this->get_logger(), "Publishing path with %zu waypoints", path.poses.size());
    path_pub_->publish(path);
  }

  rclcpp_action::Client<AStarPlanner>::SharedPtr a_star_client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  bool has_pose_;
  bool has_goal_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AStarPathClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
