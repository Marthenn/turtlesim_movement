#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "interfaces/srv/calculate_angle.hpp"
#include "PoseSubscriber.cpp"

class ThetaService : public rclcpp::Node {
public:
  ThetaService() : Node("theta_service") {
    pose_subscriber_ = std::make_shared<PoseSubscriber>();
    service_ = this->create_service<interfaces::srv::CalculateAngle>(
        "calculate_angle", std::bind(&ThetaService::callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void callback(const std::shared_ptr <interfaces::srv::CalculateAngle::Request> request,
                std::shared_ptr <interfaces::srv::CalculateAngle::Response> response) {
    float curr_x = pose_subscriber_->getX();
    float curr_y = pose_subscriber_->getY();
    float goal_x = request->x;
    float goal_y = request->y;

    // handling for division by zero and third quadrant (returned in radian)
    if (goal_x - curr_x == 0) {
      if (goal_y - curr_y > 0) {
        response->theta = 1.5708;
      } else {
        response->theta = -1.5708;
      }
    } else {
      response->theta = atan((goal_y - curr_y) / (goal_x - curr_x));
    }

    if (goal_x - curr_x < 0 && goal_y - curr_y < 0) {
      response->theta += 3.14159;
    }

    if (response->theta < 0) {
      response->theta += 6.28319;
    }
    if (response->theta > 6.28319) {
      response->theta -= 6.28319;
    }
  }

  rclcpp::Service<interfaces::srv::CalculateAngle>::SharedPtr service_;
  std::shared_ptr <PoseSubscriber> pose_subscriber_;
};
