#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class StaticPathPublisher : public rclcpp::Node {
public:
  StaticPathPublisher() : Node("static_path_publisher") {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&StaticPathPublisher::publish_path, this));
    // Define a simple straight path
    for (int i = 0; i < 20; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "odom";
      pose.pose.position.x = 0.0;
      pose.pose.position.y = i * 0.5;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path_.poses.push_back(pose);
    }
    path_.header.frame_id = "odom";
  }
private:
  void publish_path() {
    path_.header.stamp = this->now();
    for (auto &pose : path_.poses) {
      pose.header.stamp = this->now();
    }
    path_pub_->publish(path_);
  }
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path path_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticPathPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
