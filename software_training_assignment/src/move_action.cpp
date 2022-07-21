#include <software_training_assignment/move_action.hpp>

#include <thread>
#include <cmath>

namespace composition {

MoveAction::MoveAction(const rclcpp::NodeOptions &options): Node("action_server_node", options)
{
    using namespace std::placeholders;
    move_server = rclcpp_action::create_server<motion_action>(
        this,
        "waypoint_move",
        std::bind(&MoveAction::handle_goal, this, _1, _2),
        std::bind(&MoveAction::handle_cancel, this, _1),
        std::bind(&MoveAction::handle_accepted, this, _1));
    
    do_move_pub = create_publisher<twist_msg>("/moving_turtle/cmd_vel", 10);
    pos_sub = create_subscription<pose_msg>("/moving_turtle/pose", 10, std::bind(&MoveAction::get_pos, this, _1));
}

void MoveAction::get_pos(const pose_msg::SharedPtr msg) {
    // receive the messages being published by the /pose topic
    curr_pos.x = msg->x;
    curr_pos.y = msg->y;
}

float MoveAction::euc_dist() {
    float x_dist = goal_pos.x - curr_pos.x;
    float y_dist = goal_pos.y - curr_pos.y;
    return std::sqrt(std::pow(x_dist, 2) + std::pow(y_dist, 2));
}


rclcpp_action::GoalResponse MoveAction::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const motion_action::Goal> goal) 
{
    // accept all goals - could also do error checking 
    RCLCPP_INFO(this->get_logger(), "Recevied goal request with waypoint x: %f and y: %f", goal->x_pos, goal->y_pos);
    goal_pos.x = goal->x_pos;
    goal_pos.y = goal->y_pos;
    (void)uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}
    
rclcpp_action::CancelResponse MoveAction::handle_cancel(const std::shared_ptr<goal_handle_ns> goal_handle) 
{
    RCLCPP_INFO(this->get_logger(), "Request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}
    
void MoveAction::handle_accepted(const std::shared_ptr<goal_handle_ns> goal_handle)
{
    // create a thread to return quickly and not block
    std::thread{std::bind(&MoveAction::execute, this, std::placeholders::_1), goal_handle}.detach();
}
    
void MoveAction::execute(const std::shared_ptr<goal_handle_ns> goal_handle)
{
    // get node current time
    rclcpp::Time start_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Executing goal to x: %f and y: %f", goal_pos.x, goal_pos.y);
    // set frequency for goal execution (turtle to move)
    rclcpp::Rate action_rate(1);
    auto feedback_ptr = std::make_shared<motion_action::Feedback>();
    // make a reference to the distance to easily change the distance for the feedback ptr
    auto & feedback_dist = feedback_ptr->dist_to_goal;
    // setup the result (time taken)
    auto res_ptr = std::make_shared<motion_action::Result>();
    auto &res_time = res_ptr->time_taken;

    // loop here
    while(rclcpp::ok() && euc_dist() > SMALL_DIST) {
        // handle cancel action
        if (goal_handle->is_canceling()) {
            rclcpp::Time curr_time = this->now();
            rclcpp::Duration partial_time_taken = curr_time - start_time;
            // convert time to uint64
            long int partial_duration_ns = partial_time_taken.nanoseconds();
            res_time = partial_duration_ns * std::pow(10, -9);
            // set final result
            goal_handle->canceled(res_ptr);
            RCLCPP_INFO(this->get_logger(), "Action cancelled");
            return;
        }
        // move the turtle in the correct direction (calculate the movement 'slope')
        // note: this works assuming theta=0 so we are in normal XY reference frame
        float x_scaled = (goal_pos.x - curr_pos.x)*MOVE_SCALE;
        float y_scaled = (goal_pos.y - curr_pos.y)*MOVE_SCALE;
        auto move_msg = twist_msg();
        move_msg.linear.x = x_scaled;
        move_msg.linear.y = y_scaled;
        move_msg.linear.z = 0;
        move_msg.angular.x = 0;
        move_msg.angular.y = 0;
        move_msg.angular.z = 0;
        do_move_pub->publish(move_msg);

        // update the distance feedback
        feedback_dist = euc_dist();
        goal_handle->publish_feedback(feedback_ptr);
        // stall the main action loop before continuing
        action_rate.sleep();
    }

    // handle action completion
    if (rclcpp::ok()) {
        rclcpp::Time end = this->now();
        rclcpp::Duration time_taken = end - start_time;
        // convert time to uint64
        long int duration_ns = time_taken.nanoseconds();
        res_time = duration_ns * std::pow(10, -9);

        // set final result
        goal_handle->succeed(res_ptr);
        RCLCPP_INFO(this->get_logger(), "Action completed");
    }

}

}


#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::MoveAction)