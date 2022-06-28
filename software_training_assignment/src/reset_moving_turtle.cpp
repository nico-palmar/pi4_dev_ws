#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <software_training_assignment/visibility.h>
#include <std_srvs/srv/trigger.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {
    class MotionReset: public rclcpp::Node {
    
    public: 
        using teleport_srv = turtlesim::srv::TeleportAbsolute;
        using trigger_srv = std_srvs::srv::Trigger;

        SOFTWARE_TRAINING_PUBLIC
        explicit MotionReset(const rclcpp::NodeOptions &options): rclcpp::Node("reset_moving_turle_node", options) {
            reset_moving_server = create_service<trigger_srv>("moving_turtle_reset", std::bind(&MotionReset::pos_reset, this, _1, _2));
            teleport_abs_client = create_client<teleport_srv>("/moving_turle/teleport_absolute");
        }
    private:
        rclcpp::Service<trigger_srv>::SharedPtr reset_moving_server;
        rclcpp::Client<teleport_srv>::SharedPtr teleport_abs_client;

        // create a struct that holds the position in a shared header file (define and declare). Include here and turtles.cpp
        // also in that header file, add the ros client checking function. Call it common.hpp maybe?


        SOFTWARE_TRAINING_LOCAL
        void pos_reset(const std::shared_ptr<trigger_srv::Request> req, std::shared_ptr<trigger_srv::Response> resp) {

        }

    };
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::MotionReset)