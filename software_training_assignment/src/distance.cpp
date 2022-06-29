
#include <software_training_assignment/distance.hpp>
#include <software_training_assignment/common.hpp>
#include <cmath>

// using pose_msg = turtlesim::msg::Pose;
// using distance_msg = software_training_assignment::msg::Distances;
using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {

DistanceInfo::DistanceInfo(const rclcpp::NodeOptions &options): Node("distances_node", options) {
    distance_pub = create_publisher<distance_msg>("turtle_distances", 10);
    moving_pos_sub = create_subscription<pose_msg>("moving_turtle/pose", 10, std::bind(&DistanceInfo::get_moving_position, this, _1));
    pub_timer = create_wall_timer(500ms, std::bind(&DistanceInfo::publish_distances, this));
    read_move = false;
    // copy the init position for the global variable
    Position current_moving_turtle_pos = { init_moving_turtle_pos.x, init_moving_turtle_pos.y, init_moving_turtle_pos.theta };
}

void DistanceInfo::publish_distances() {
    if (read_move == true) {
        // compute the distances (stationary to moving)
        float x_dist = current_moving_turtle_pos.x - stationary_turtle_pos.x;
        float y_dist = current_moving_turtle_pos.y - stationary_turtle_pos.y;
        double euc_dist = compute_euc_dist(current_moving_turtle_pos, stationary_turtle_pos);

        // publish distance message
        auto dist_msg = distance_msg();
        dist_msg.x_dist = x_dist;
        dist_msg.y_dist = y_dist;
        dist_msg.euc_dist = euc_dist;
        RCLCPP_INFO(this->get_logger(), "Publishing distances: x %f | y %f | euclidean %f", dist_msg.x_dist, dist_msg.y_dist, dist_msg.euc_dist);
        distance_pub->publish(dist_msg);

    } else {
        RCLCPP_WARN(this->get_logger(), "No position for moving turtle, distances not computed");
    }
}

void DistanceInfo::get_moving_position(const pose_msg::SharedPtr msg) {
    // store the moving turtle position
    current_moving_turtle_pos.x = msg->x;
    current_moving_turtle_pos.y = msg->y;
    current_moving_turtle_pos.theta = msg->theta;
    RCLCPP_INFO(this->get_logger(), "Read current moving pos: x %f | y %f | theta %f", current_moving_turtle_pos.x, current_moving_turtle_pos.y, current_moving_turtle_pos.theta);
    read_move = true;
}

double DistanceInfo::compute_euc_dist(Position p2, Position p1) {
    double x_euc = std::pow((p2.x - p1.x), 2);
    double y_euc = std::pow((p2.y - p1.y), 2);
    double res = std::sqrt(x_euc + y_euc);
    return res;
}

}


#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::DistanceInfo)