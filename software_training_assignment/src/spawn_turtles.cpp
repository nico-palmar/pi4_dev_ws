
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <software_training_assignment/visibility.h>
#include <std_srvs/srv/trigger.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <software_training_assignment/common.hpp>
// #include <turtlesim/srv/kill.hpp>


using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {

class SpawnTwoTurtles: public rclcpp::Node {
public:
    SOFTWARE_TRAINING_PUBLIC
    explicit SpawnTwoTurtles(const rclcpp::NodeOptions &options): Node("spawn_turtle_node", options) {
        spawn_client = create_client<turtlesim::srv::Spawn>("/spawn");
        // add_turtle_client = create_client<turtlesim::srv::Kill>("/add_turtle_name");
        // define the starting positions order { x, y, theta }
        two_turtle_server = create_service<std_srvs::srv::Trigger>("spawn_two_turtles", std::bind(&SpawnTwoTurtles::two_turtle_spawn, this, _1, _2));
    }

private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr two_turtle_server;
    // rclcpp::Client<turtlesim::srv::Kill>::SharedPtr add_turtle_client;
    bool did_spawn = false;

    // define positions here
    Position init_moving_turtle_pos = { 9, 8, 0 };
    Position stationary_turtle_pos = { 5, 5, 0 };

    SOFTWARE_TRAINING_LOCAL

    void two_turtle_spawn(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // if (!did_spawn) {        
        auto stationary_spawn_req = std::make_shared<turtlesim::srv::Spawn::Request>();
        stationary_spawn_req->name = "stationary_turtle";
        
        stationary_spawn_req->x = stationary_turtle_pos.x;
        stationary_spawn_req->y = stationary_turtle_pos.y;
        stationary_spawn_req->theta = stationary_turtle_pos.theta;
        auto stationary_spawn_res = spawn_client->async_send_request(stationary_spawn_req);

        auto moving_spawn_req = std::make_shared<turtlesim::srv::Spawn::Request>();
        moving_spawn_req->name = "moving_turtle";
        
        moving_spawn_req->x = init_moving_turtle_pos.x;
        moving_spawn_req->y = init_moving_turtle_pos.y;
        moving_spawn_req->theta = init_moving_turtle_pos.theta;
        auto moving_spawn_res = spawn_client->async_send_request(moving_spawn_req);

        did_spawn = true;
        response->success = true;
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "Turtle spawn service already called");
        // }
        
    }

};

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::SpawnTwoTurtles)