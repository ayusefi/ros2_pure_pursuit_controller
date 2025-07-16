#ifndef ROS2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define ROS2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>


class PurePursuitController : public rclcpp::Node {
public:
  PurePursuitController();

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void compute_and_publish_cmd();
  geometry_msgs::msg::Point find_lookahead_point(const geometry_msgs::msg::Pose &current_pose, const std::vector<geometry_msgs::msg::PoseStamped> &path, double lookahead_distance);
  std::pair<double, double> calculate_control(const geometry_msgs::msg::Pose &current_pose, const geometry_msgs::msg::Point &lookahead_point);
  double get_yaw_from_pose(const geometry_msgs::msg::Pose &pose);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  geometry_msgs::msg::Pose current_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  bool has_pose_ = false;
  bool has_path_ = false;
  double lookahead_distance_ = 0.5;
  double max_linear_speed_ = 0.5;
  double max_angular_speed_ = 1.0;

  // Add callback handle for parameter updates
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

#endif // ROS2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
