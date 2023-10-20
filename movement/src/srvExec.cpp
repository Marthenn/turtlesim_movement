#include "rclcpp/rclcpp.hpp"
#include "ThetaServer.cpp"

int main() {
  rclcpp::init(0, nullptr);
  auto theta_server = std::make_shared<ThetaService>();
  rclcpp::spin(theta_server);
  rclcpp::shutdown();
  return 0;
}