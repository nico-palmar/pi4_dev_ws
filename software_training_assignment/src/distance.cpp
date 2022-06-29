
#include <software_training_assignment/distance.hpp>
#include <software_training_assignment/common.hpp>

// using pose_msg = turtlesim::msg::Pose;
// using distance_msg = software_training_assignment::msg::Distances;
using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {

DistanceInfo::DistanceInfo(const rclcpp::NodeOptions &options): Node("distances_node", options) {
    distance_pub = create_publisher<distance_msg>("turtle_distances", 10);
    moving_pos_sub = create_subscription<pose_msg>("moving_turtle/pose", 10, std::bind(&DistanceInfo::get_moving_position, this, _1));
    pub_timer = create_wall_timer(500ms, std::bind(&DistanceInfo::publish_distances, this));
    // copy the init position for the global variable
    Position current_moving_turtle_pos = { init_moving_turtle_pos.x, init_moving_turtle_pos.y, init_moving_turtle_pos.theta };
}

void DistanceInfo::publish_distances() {
    // compute the distances (stationary to current moving)
    auto 

    // publish distance message
}

void DistanceInfo::get_moving_position(const pose_msg::SharedPtr msg) {
    // store the moving turtle position
}

}


#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::DistanceInfo)