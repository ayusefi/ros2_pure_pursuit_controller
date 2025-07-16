#include "ros2_pure_pursuit_controller/pure_pursuit_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>

PurePursuitController::PurePursuitController() : Node("pure_pursuit_controller") {
  // Declare parameters with default values
  this->declare_parameter<double>("lookahead_distance", 0.5);
  this->declare_parameter<double>("max_linear_speed", 0.5);
  this->declare_parameter<double>("max_angular_speed", 1.0);
  // Get initial values
  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
  // Set up callback for parameter changes (dynamic tuning)
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &params) {
      for (const auto &param : params) {
        if (param.get_name() == "lookahead_distance") {
          lookahead_distance_ = param.as_double();
        } else if (param.get_name() == "max_linear_speed") {
          max_linear_speed_ = param.as_double();
        } else if (param.get_name() == "max_angular_speed") {
          max_angular_speed_ = param.as_double();
        }
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "Parameters set";
      return result;
    });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/bcr_bot/odom", 10,
    std::bind(&PurePursuitController::odom_callback, this, std::placeholders::_1));
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/plan", 10,
    std::bind(&PurePursuitController::path_callback, this, std::placeholders::_1));
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/bcr_bot/cmd_vel", 10);
}


void PurePursuitController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose_ = msg->pose.pose;
  has_pose_ = true;
  compute_and_publish_cmd();
}

void PurePursuitController::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  path_ = msg->poses;
  has_path_ = true;
}

void PurePursuitController::compute_and_publish_cmd() {
  if (!has_pose_ || !has_path_ || path_.empty()) return;
  geometry_msgs::msg::Point lookahead_point = find_lookahead_point(current_pose_, path_, lookahead_distance_);
  if (std::isnan(lookahead_point.x)) {
    RCLCPP_INFO(this->get_logger(), "No lookahead point found.");
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
    return;
  }
  
  // Log current position and lookahead point for debugging
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "Robot: (%.2f, %.2f), Lookahead: (%.2f, %.2f)", 
    current_pose_.position.x, current_pose_.position.y,
    lookahead_point.x, lookahead_point.y);
  
  auto [linear, angular] = calculate_control(current_pose_, lookahead_point);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::clamp(linear, 0.0, max_linear_speed_);
  cmd.angular.z = std::clamp(angular, -max_angular_speed_, max_angular_speed_);
  cmd_pub_->publish(cmd);
}

geometry_msgs::msg::Point PurePursuitController::find_lookahead_point(
  const geometry_msgs::msg::Pose &current_pose,
  const std::vector<geometry_msgs::msg::PoseStamped> &path,
  double lookahead_distance)
{
  geometry_msgs::msg::Point nan_point;
  nan_point.x = nan_point.y = nan_point.z = std::numeric_limits<double>::quiet_NaN();
  double px = current_pose.position.x;
  double py = current_pose.position.y;
  
  // Find closest point on path first
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.size(); ++i) {
    double dist = std::hypot(path[i].pose.position.x - px, path[i].pose.position.y - py);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  
  // Search forward from closest point for lookahead distance
  for (size_t i = closest_idx; i < path.size(); ++i) {
    double dist = std::hypot(path[i].pose.position.x - px, path[i].pose.position.y - py);
    if (dist >= lookahead_distance) {
      geometry_msgs::msg::Point pt;
      pt.x = path[i].pose.position.x;
      pt.y = path[i].pose.position.y;
      pt.z = 0.0;
      return pt;
    }
  }
  
  // If no point found at lookahead distance, return last point (goal)
  if (!path.empty()) {
    geometry_msgs::msg::Point pt;
    pt.x = path.back().pose.position.x;
    pt.y = path.back().pose.position.y;
    pt.z = 0.0;
    return pt;
  }
  
  return nan_point;
}

std::pair<double, double> PurePursuitController::calculate_control(
  const geometry_msgs::msg::Pose &current_pose,
  const geometry_msgs::msg::Point &lookahead_point)
{
  double px = current_pose.position.x;
  double py = current_pose.position.y;
  double dx = lookahead_point.x - px;
  double dy = lookahead_point.y - py;
  double yaw = get_yaw_from_pose(current_pose);
  double lx = std::cos(-yaw) * dx - std::sin(-yaw) * dy;
  double ly = std::sin(-yaw) * dx + std::cos(-yaw) * dy;
  double ld = std::hypot(lx, ly);
  
  // Stop if we're very close to the goal
  if (ld < 0.1) return {0.0, 0.0};
  
  // Calculate angle to target
  double angle_to_target = std::atan2(ly, lx);
  
  // For differential drive: if target is significantly behind or requires sharp turn,
  // rotate in place first
  if (lx < 0 || std::abs(angle_to_target) > M_PI/3) {
    // Pure rotation - use sign of ly to determine direction
    double angular = (ly > 0 ? 1.0 : -1.0) * max_angular_speed_ * 0.7;
    return {0.0, angular};
  }
  
  // Forward motion with steering - differential drive can handle gradual turns while moving
  double curvature = 2 * ly / (ld * ld);
  double linear = max_linear_speed_;
  
  // Reduce speed for sharper turns (differential drive benefit)
  if (std::abs(curvature) > 1.0) {
    linear *= 0.5;
  }
  
  double angular = curvature * linear;
  return {linear, angular};
}

double PurePursuitController::get_yaw_from_pose(const geometry_msgs::msg::Pose &pose) {
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuitController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
