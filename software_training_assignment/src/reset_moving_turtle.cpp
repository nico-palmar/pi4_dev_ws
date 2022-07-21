#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <software_training_assignment/visibility.h>
#include <std_srvs/srv/trigger.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <software_training_assignment/common.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

// TODO: test this to make sure it works
namespace composition {
    class MotionReset: public rclcpp::Node {
    
    public: 
        using teleport_srv = turtlesim::srv::TeleportAbsolute;
        using trigger_srv = std_srvs::srv::Trigger;

        SOFTWARE_TRAINING_PUBLIC
        explicit MotionReset(const rclcpp::NodeOptions &options): rclcpp::Node("reset_moving_turle_node", options) {
            reset_moving_server = create_service<trigger_srv>("moving_turtle_reset", std::bind(&MotionReset::pos_reset, this, _1, _2));
            teleport_abs_client = create_client<teleport_srv>("/moving_turtle/teleport_absolute");
        }
    private:
        rclcpp::Service<trigger_srv>::SharedPtr reset_moving_server;
        rclcpp::Client<teleport_srv>::SharedPtr teleport_abs_client;
        Position init_moving_turtle_pos = { 9, 8, 0 }; // starting position for the moving turtle


        SOFTWARE_TRAINING_LOCAL
        void pos_reset(const std::shared_ptr<trigger_srv::Request> /* req */, std::shared_ptr<trigger_srv::Response> resp) {
            if (!teleport_abs_client->wait_for_service(2s)) {
                if(!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for teleport_absolute service, exiting");
                    resp->success = false;
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Service teleport_absolute not avaliable");
                resp->success = false;
                return;
            }
            auto teleport_req = std::make_shared<teleport_srv::Request>();
            teleport_req->x = init_moving_turtle_pos.x;
            teleport_req->y = init_moving_turtle_pos.y;
            teleport_req->theta = init_moving_turtle_pos.theta;
            auto res = teleport_abs_client->async_send_request(teleport_req);
            resp->success = true;
            resp->message = "Moving turtle position reset";
            return;
        }

    };
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::MotionReset)