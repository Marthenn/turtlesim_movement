#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "interfaces/srv/calculate_angle.hpp"
#include "interfaces/action/coordinate.hpp"
#include "turtlesim/action/rotate_absolute.hpp"
#include "PoseSubscriber.cpp"
#include "MovementPublisher.cpp"

#include "movement/visibility_control.h"

namespace action_coordinate_cpp {
  class CoordinateAction : public rclcpp::Node {
  public:
    ACTION_COORDINATE_CPP_PUBLIC
    explicit CoordinateAction(const rclcpp::NodeOptions &options)
        : Node("coordinate_action", options) {
      action_server_ = rclcpp_action::create_server<interfaces::action::Coordinate>(
          this,
          "coordinate",
          std::bind(&CoordinateAction::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&CoordinateAction::handle_cancel, this, std::placeholders::_1),
          std::bind(&CoordinateAction::handle_accepted, this, std::placeholders::_1)
      );
      theta_client_ = this->create_client<interfaces::srv::CalculateAngle>("calculate_angle");
      rotate_client_ = rclcpp_action::create_client<turtlesim::action::RotateAbsolute>(this,
                                                                                       "/turtle1/rotate_absolute");
    }

  private:
    rclcpp_action::Server<interfaces::action::Coordinate>::SharedPtr action_server_;
    rclcpp::Client<interfaces::srv::CalculateAngle>::SharedPtr theta_client_;
    rclcpp_action::Client<turtlesim::action::RotateAbsolute>::SharedPtr rotate_client_;
    std::shared_ptr <PoseSubscriber> pose_subscriber_;
    std::shared_ptr <MovementPublisher> movement_publisher_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const interfaces::action::Coordinate::Goal> goal) {
      RCLCPP_INFO(this->get_logger(), "Received goal request with x: %f, y: %f", goal->x, goal->y);
      (void) uuid;
      if (!validateGoal(goal->x, goal->y)) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr <rclcpp_action::ServerGoalHandle<interfaces::action::Coordinate>> goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void) goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr <rclcpp_action::ServerGoalHandle<interfaces::action::Coordinate>> goal_handle) {
      std::thread{std::bind(&CoordinateAction::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr <rclcpp_action::ServerGoalHandle<interfaces::action::Coordinate>> goal_handle) {
      pose_subscriber_ = std::make_shared<PoseSubscriber>();
      movement_publisher_ = std::make_shared<MovementPublisher>();

      // set parameter of linear velocity
      movement_publisher_->set_parameter(rclcpp::Parameter("linear_velocity", 0.5)); // slow = easier to control
      movement_publisher_->set_parameter(rclcpp::Parameter("angular_velocity", 0)); // no angular velocity

      RCLCPP_INFO(this->get_logger(), "Executing goal");
      rclcpp::Rate loop_rate(1);
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<interfaces::action::Coordinate::Feedback>();
      auto result = std::make_shared<interfaces::action::Coordinate::Result>();

      // get theta from service
      auto request = std::make_shared<interfaces::srv::CalculateAngle::Request>();
      request->x = goal->x;
      request->y = goal->y;
      auto theta = theta_client_->async_send_request(request).get();

      if (!theta) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get theta");
        result->success = false;
        goal_handle->succeed(result);
        return;
      }

      float rotation = theta->theta;

      // rotate to theta using service
      auto rotate_goal = turtlesim::action::RotateAbsolute::Goal();
      rotate_goal.theta = rotation;
      RCLCPP_INFO(this->get_logger(), "Rotating to theta: %f", rotation);
      auto rotate_result = rotate_client_->async_send_goal(rotate_goal).get();

      if (!rotate_result) {
        RCLCPP_ERROR(this->get_logger(), "Failed to rotate");
        result->success = false;
        goal_handle->succeed(result);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Rotated to theta: %f", rotation);
      RCLCPP_INFO(this->get_logger(), "Moving to x: %f, y: %f", goal->x, goal->y);

      // move to goal
      while (!isGoalReached(pose_subscriber_->getX(), pose_subscriber_->getY(), goal->x, goal->y)) {
        RCLCPP_INFO(this->get_logger(), "Current x: %f, y: %f", pose_subscriber_->getX(), pose_subscriber_->getY());
        movement_publisher_->publish();
        feedback->x = pose_subscriber_->getX();
        feedback->y = pose_subscriber_->getY();
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
      }

      // stop moving
      movement_publisher_->set_parameter(rclcpp::Parameter("linear_velocity", 0));
      movement_publisher_->set_parameter(rclcpp::Parameter("angular_velocity", 0));

      // set result
      result->success = true;
      goal_handle->succeed(result);
    }

    bool validateGoal(float x, float y) {
      return x >= 0 && y >= 0 && x <= 11 && y <= 11;
    }

    bool isGoalReached(float curr_x, float curr_y, float goal_x, float goal_y) {
      return abs(curr_x - goal_x) < 0.2 && abs(curr_y - goal_y) < 0.2;
    }
  };
}

RCLCPP_COMPONENTS_REGISTER_NODE(action_coordinate_cpp::CoordinateAction);