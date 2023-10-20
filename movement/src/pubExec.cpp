#include "rclcpp/rclcpp.hpp"
#include "MovementPublisher.cpp"

int main() {
  rclcpp::init(0, nullptr);
  auto movement_publisher = std::make_shared<MovementPublisher>();
  rclcpp::spin(movement_publisher);
  rclcpp::shutdown();
  return 0;
}