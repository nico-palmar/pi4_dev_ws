#ifndef DISTANCE_H
#define DISTANCE_H

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <software_training_assignment/visibility.h>
#include <turtlesim/msg/pose.hpp>
#include <software_training_assignment/msg/distances.hpp>

// using namespace std::chrono_literals;
// using namespace std::placeholders;


namespace composition {
    class DistanceInfo: public rclcpp::Node {
    
    public: 
        using pose_msg = turtlesim::msg::Pose;
        using distance_msg = software_training_assignment::msg::Distances;

        SOFTWARE_TRAINING_PUBLIC
        explicit DistanceInfo(const rclcpp::NodeOptions &options);

    private:
        rclcpp::Publisher<distance_msg>::SharedPtr distance_pub;
        rclcpp::TimerBase::SharedPtr pub_timer;
        rclcpp::Subscription<pose_msg>::SharedPtr moving_pos_sub;
        bool read_move;

        SOFTWARE_TRAINING_LOCAL
        // timer call back to get current positions and compute distances
        void publish_distances();
        // subscriber function to get data from topic
        void get_moving_position(const pose_msg::SharedPtr msg);

        // helper function for computing euclidean distance
        double compute_euc_dist(Position p2, Position p1);

    };
}

#endif