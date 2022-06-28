
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <software_training_assignment/visibility.h>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {
    class CircularMotion: public rclcpp::Node {
        using trigger_srv = std_srvs::srv::Trigger;
        using twist_msg = geometry_msgs::msg::Twist;
    
        public:
            SOFTWARE_TRAINING_PUBLIC
            explicit CircularMotion(const rclcpp::NodeOptions &options): Node("circular_motion_node", options) {
                move_server = create_service<trigger_srv>("move_turtle1_circular", std::bind(&CircularMotion::move_circular, this, _1, _2));
                move_publisher = create_publisher<twist_msg>("turtle1/cmd_vel", 10);

            }
        private:
            rclcpp::Service<trigger_srv>::SharedPtr move_server; // use the service to start the timer
            rclcpp::Publisher<twist_msg>::SharedPtr move_publisher;
            rclcpp::TimerBase::SharedPtr move_timer;

            SOFTWARE_TRAINING_LOCAL
            void move_circular(const std::shared_ptr<trigger_srv::Request> /* req */, std::shared_ptr<trigger_srv::Response> resp) {
                // create the timer when the service is called
                RCLCPP_INFO(this->get_logger(), "Creating turtle1 publishing timer");
                move_timer = create_wall_timer(1000ms, std::bind(&CircularMotion::timer_callback, this));
                resp->success = true;
            }

            void timer_callback() {
                // create the message
                auto circle_msg = twist_msg();
                circle_msg.linear.x = 1;
                circle_msg.linear.y = 0;
                circle_msg.linear.z = 0;
                circle_msg.angular.x = 0;
                circle_msg.angular.y = 0;
                circle_msg.angular.z = 1;
                // consistently publish data to move in a circle (every second)
                move_publisher->publish(circle_msg);
            }
    };
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::CircularMotion)

