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
    float x = pose_subscriber_->getX();
    float y = pose_subscriber_->getY();

    float deltaX = request->x - x;
    float deltaY = request->y - y;

    response->theta = atan2(deltaY, deltaX);
  }

  rclcpp::Service<interfaces::srv::CalculateAngle>::SharedPtr service_;
  std::shared_ptr <PoseSubscriber> pose_subscriber_;
};
