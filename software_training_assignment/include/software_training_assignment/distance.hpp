#ifndef DISTANCE_H
#define DISTANCE_H

#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <software_training_assignment/visibility.h>
#include <turtlesim/msg/pose.hpp>
#include <software_training_assignment/msg/distances.hpp>
#include <software_training_assignment/common.hpp>

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
        rclcpp::Subscription<pose_msg>::SharedPtr static_pos_sub;

        // create threads to run callbacks
        rclcpp::CallbackGroup::SharedPtr callbacks;
        
        // struct Position {
        //     float x;
        //     float y;
        //     float theta;
        // };
        
        Position moving_turtle_pos;
        Position static_turtle_pos;
        bool read_moving, read_static;

        SOFTWARE_TRAINING_LOCAL
        // timer call back to get current positions and compute distances
        void publish_distances();
        // subscriber functions to get data from topic
        void get_moving_position(const pose_msg::SharedPtr msg);
        void get_static_position(const pose_msg::SharedPtr msg);

        // helper function for computing euclidean distance
        double compute_euc_dist();

    };
}

#endif