
#include <software_training_assignment/distance.hpp>
#include <cmath>

// using pose_msg = turtlesim::msg::Pose;
// using distance_msg = software_training_assignment::msg::Distances;
using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {

DistanceInfo::DistanceInfo(const rclcpp::NodeOptions &options): Node("distances_node", options) {
    // make mutex for subscribers and timer (since callback functions should hold lock)
    callbacks = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // callback thread
    auto callback_options = rclcpp::SubscriptionOptions();
    callback_options.callback_group = callbacks;
    distance_pub = create_publisher<distance_msg>("turtle_distances", 10);
    moving_pos_sub = create_subscription<pose_msg>("moving_turtle/pose", 10, std::bind(&DistanceInfo::get_moving_position, this, _1), callback_options);
    static_pos_sub = create_subscription<pose_msg>("stationary_turtle/pose", 10, std::bind(&DistanceInfo::get_static_position, this, _1), callback_options);
    pub_timer = create_wall_timer(500ms, std::bind(&DistanceInfo::publish_distances, this), callbacks);
    read_moving = false;
    read_static = false;
}

void DistanceInfo::publish_distances() {
    if (read_moving && read_static) {
        // compute the distances (stationary to moving)
        float x_dist = moving_turtle_pos.x - static_turtle_pos.x;
        float y_dist = moving_turtle_pos.y - static_turtle_pos.y;
        double euc_dist = compute_euc_dist();

        // publish distance message
        auto dist_msg = distance_msg();
        dist_msg.x_dist = x_dist;
        dist_msg.y_dist = y_dist;
        dist_msg.euc_dist = euc_dist;
        RCLCPP_INFO(this->get_logger(), "Publishing distances: x %f | y %f | euclidean %f", dist_msg.x_dist, dist_msg.y_dist, dist_msg.euc_dist);
        distance_pub->publish(dist_msg);

    } else {
        RCLCPP_WARN(this->get_logger(), "No position for either moving or static turtle, distances not computed");
    }
}

void DistanceInfo::get_moving_position(const pose_msg::SharedPtr msg) {
    // store the moving turtle position
    moving_turtle_pos.x = msg->x;
    moving_turtle_pos.y = msg->y;
    moving_turtle_pos.theta = msg->theta;
    // RCLCPP_INFO(this->get_logger(), "Read moving pos: x %f | y %f | theta %f", moving_turtle_pos.x, moving_turtle_pos.y, moving_turtle_pos.theta);
    read_moving = true;
}

void DistanceInfo::get_static_position(const pose_msg::SharedPtr msg) {
    // store the stationary turtle position
    static_turtle_pos.x = msg->x;
    static_turtle_pos.y = msg->y;
    static_turtle_pos.theta = msg->theta;
    // RCLCPP_INFO(this->get_logger(), "Read static pos: x %f | y %f | theta %f", static_turtle_pos.x, static_turtle_pos.y, static_turtle_pos.theta);
    read_static = true;
}

double DistanceInfo::compute_euc_dist() {
    double x_euc = std::pow((moving_turtle_pos.x - static_turtle_pos.x), 2);
    double y_euc = std::pow((moving_turtle_pos.y - static_turtle_pos.y), 2);
    double res = std::sqrt(x_euc + y_euc);
    return res;
}

}


#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::DistanceInfo)