
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <software_training_assignment/visibility.h>
#include <turtlesim/srv/kill.hpp>

using namespace std::chrono_literals;

namespace composition {

class clear_turtles : public rclcpp::Node {
public:
  SOFTWARE_TRAINING_PUBLIC
  explicit clear_turtles(const rclcpp::NodeOptions &options): Node("clear_turtle_node", options) {
      // create the client to call the kill service
      client = create_client<turtlesim::srv::Kill>("/kill");
  }

private:
  rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client;

  // all the turtles including the default turtle
  std::vector<std::string> turtle_names = {"turtle1", "moving_turtle",
                                           "stationary_turtle"};

  SOFTWARE_TRAINING_LOCAL

  void kill_callback(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture resp) {
      (void)resp;
      RCLCPP_INFO(this->get_logger(), "Turtle killed!");
      // stop executing callback
      rclcpp::shutdown();
  }
  void kill() {
    if (!client->wait_for_service(2s)) {
          if(!rclcpp::ok()) {
              RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for kill service, exiting");
              return;
          }
          RCLCPP_INFO(this->get_logger(), "Service not avaliable");
          return;
    }

    for (auto name: turtle_names) {
        using namespace std::placeholders;
        auto req = std::make_shared<turtlesim::srv::Kill::Request>();
        req->name = name;
        auto res = client->async_send_request(req, std::bind(&clear_turtles::kill_callback, this, _1));
    }
    

    
  }
};

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::clear_turtles)
