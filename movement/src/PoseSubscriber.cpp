#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

class PoseSubscriber : public rclcpp::Node {
public:
  PoseSubscriber() : Node("pose_subscriber") {
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10, std::bind(&PoseSubscriber::callback, this, std::placeholders::_1));
  }

  float getX() { return x; }
  float getY() { return y; }
  float getTheta() { return theta; }

private:
  float x, y, theta;
  void callback(const turtlesim::msg::Pose::SharedPtr msg) {
    x = msg->x;
    y = msg->y;
    theta = msg->theta;
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(){
  rclcpp::init(0, nullptr);
  rclcpp::spin(std::make_shared<PoseSubscriber>());
  rclcpp::shutdown();
  return 0;
}