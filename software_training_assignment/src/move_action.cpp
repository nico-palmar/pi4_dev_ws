#include <software_training_assignment/move_action.hpp>

#include <thread>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {

MoveAction::MoveAction(const rclcpp::NodeOptions &options) 
{

}



rclcpp_action::GoalResponse MoveAction::handle_goal(
    const rclcpp_actin::GoalUUID &uuid,
    std::shared_ptr<const motion_action::Goal> goal) 
{


}
    
rclcpp_action::CancelResponse MoveAction::handle_cancel(const std::shared_ptr<goal_handle_ns> goal_handle) 
{
    
}
    
void MoveAction::handle_accepted(const std::shared_ptr<goal_handle_ns> goal_handle)
{

}
    
void MoveAction::execute(const std::shared_ptr<goal_handle_ns> goal_handle)
{

}

}


#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::MoveAction)