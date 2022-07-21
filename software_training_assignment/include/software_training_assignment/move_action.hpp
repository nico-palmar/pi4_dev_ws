#ifndef DISTANCE_H
#define DISTANCE_H

#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <software_training_assignment/visibility.h>
#include <software_training_assignment/common.hpp>
#include "software_training_assignment/action/move.hpp"


namespace composition {
class MoveAction: public rclcpp::Node {
public: 
    using motion_action = software_training_assignment::action::Move;
    using motion_server = rclcpp_action::Server<motion_action>;
    using goal_handle_ns = rclcpp_action::ServerGoalHandle<motion_action>;
    using pose_msg = turtlesim::msg::Pose;
    using twist_msg = geometry_msgs::msg::Twist;
    SOFTWARE_TRAINING_PUBLIC
    explicit MoveAction(const rclcpp::NodeOptions &options);

private:
    motion_server::SharedPtr move_server;
    rclcpp::Publisher<twist_msg>::SharedPtr do_move_pub;
    rclcpp::Subscription<pose_msg>::SharedPtr pos_sub;

    Position curr_pos;
    Position goal_pos;
    // constant values to play around with
    const float SMALL_DIST = 0.1;
    const float MOVE_SCALE = 0.5;

    void get_pos(const pose_msg::SharedPtr msg);
    float euc_dist();

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const motion_action::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<goal_handle_ns> goal_handle);
    
    void handle_accepted(const std::shared_ptr<goal_handle_ns> goal_handle);
    
    void execute(const std::shared_ptr<goal_handle_ns> goal_handle);

};
}

#endif