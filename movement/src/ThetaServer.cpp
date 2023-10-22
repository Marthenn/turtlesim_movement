#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "interfaces/srv/calculate_angle.hpp"
#include "PoseSubscriber.cpp"

class ThetaService : public rclcpp::Node {
public:
  ThetaService() : Node("theta_service") {
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10, std::bind(&ThetaService::pose_callback, this, std::placeholders::_1));
    service_ = this->create_service<interfaces::srv::CalculateAngle>(
        "calculate_angle", std::bind(&ThetaService::callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  float x, y;

  void pose_callback(const turtlesim::msg::Pose::SharedPtr pose) {
    x = pose->x;
    y = pose->y;
  }

  void callback(const std::shared_ptr <interfaces::srv::CalculateAngle::Request> request,
                std::shared_ptr <interfaces::srv::CalculateAngle::Response> response) {
    float goal_x = request->x;
    float goal_y = request->y;

    RCLCPP_INFO(this->get_logger(), "Received request with x: %f, y: %f", goal_x, goal_y);
    RCLCPP_INFO(this->get_logger(), "Current position is x: %f, y: %f", this->x, this->y);

    // handling for division by zero and third quadrant (returned in radian)
    if (goal_x - this->x == 0) {
      if (goal_y - this->y > 0) {
        response->theta = 1.5708;
      } else {
        response->theta = -1.5708;
      }
    } else {
      response->theta = atan((goal_y - this->y) / (goal_x - this->x));
    }

    if (goal_x - this->x < 0 && goal_y - this->y < 0) {
      response->theta += 3.14159;
    }

    if (response->theta < 0) {
      response->theta += 6.28319;
    }
    if (response->theta > 6.28319) {
      response->theta -= 6.28319;
    }

    RCLCPP_INFO(this->get_logger(), "Sending response with theta: %f", response->theta);
  }

  rclcpp::Service<interfaces::srv::CalculateAngle>::SharedPtr service_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};
