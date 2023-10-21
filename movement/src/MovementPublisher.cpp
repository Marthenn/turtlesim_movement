#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MovementPublisher : public rclcpp::Node {
public:
  MovementPublisher() : Node("movement_publisher") {
    // parameters for the linear and angular velocities
    this->declare_parameter("linear_velocity", 1.0);
    this->declare_parameter("angular_velocity", 1.0);

    // create a publisher for the velocities
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

    // create a timer that will publish the velocities every 1 second
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MovementPublisher::timer_callback, this));
  }

  void publish() {
    timer_callback();
  }

private:
  void timer_callback() {
    // create a message for the velocities
    auto message = geometry_msgs::msg::Twist();

    // get the parameters for the velocities
    double linear_velocity = this->get_parameter("linear_velocity").as_double();
    double angular_velocity = this->get_parameter("angular_velocity").as_double();

    // set the velocities in the message
    message.linear.x = linear_velocity;
    message.angular.z = angular_velocity;

    // publish the message
    publisher_->publish(std::move(message));
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};
