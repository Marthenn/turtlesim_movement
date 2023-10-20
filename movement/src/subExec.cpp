#include "PoseSubscriber.cpp"
#include "rclcpp/rclcpp.hpp"

int main() {
  rclcpp::init(0, nullptr);
  auto pose_subscriber = std::make_shared<PoseSubscriber>();
  rclcpp::spin(pose_subscriber);
  rclcpp::shutdown();
  return 0;
}