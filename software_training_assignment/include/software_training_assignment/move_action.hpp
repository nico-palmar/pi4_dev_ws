#ifndef DISTANCE_H
#define DISTANCE_H

#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <software_training_assignment/visibility.h>
#include <software_training_assignment/common.hpp>
#include "software_training_assignment/action/move.hpp"


namespace composition {
class MoveAction: public rclcpp::Node {
public: 
    using motion_action = software_training_assignment::action::Move;
    using motion_server = rclcpp_action::Server<motion_action>;
    using goal_handle_ns = rclcpp_action::ServerGoalHandle<motion_action>;
    SOFTWARE_TRAINING_PUBLIC
    explicit MoveAction(const rclcpp::NodeOptions &options);

private:
    motion_server::SharedPtr move_server;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_actin::GoalUUID &uuid,
        std::shared_ptr<const motion_action::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<goal_handle_ns> goal_handle);
    
    void handle_accepted(const std::shared_ptr<goal_handle_ns> goal_handle);
    
    void execute(const std::shared_ptr<goal_handle_ns> goal_handle);

};
}

#endif