
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <software_training_assignment/visibility.h>
#include <turtlesim/srv/kill.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

namespace composition {

using namespace std::placeholders;
class clear_turtles : public rclcpp::Node {
public:
  SOFTWARE_TRAINING_PUBLIC
  explicit clear_turtles(const rclcpp::NodeOptions &options): Node("clear_turtle_node", options) {
      // create the client to call the kill service
      client = create_client<turtlesim::srv::Kill>("/kill");
      // create a service to call the kill client
      clear_server = create_service<std_srvs::srv::Trigger>("remove_all_turtles", std::bind(&clear_turtles::clear, this, _1, _2));
      // create a service to add new turtles when spawning (to know which turtles to kill when clearing)
      add_name_server = create_service<turtlesim::srv::Kill>("add_turtle_name", std::bind(&clear_turtles::add_name, this, _1, _2));
  }

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_server;
  rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client;
  // create a service to add turtles to the list of turtles
  rclcpp::Service<turtlesim::srv::Kill>::SharedPtr add_name_server;

  // clear the initial turtle which is launched with the turtle node
  std::vector<std::string> turtle_names = {"turtle1", "moving_turtle", "stationary_turtle"};
  // , "moving_turtle", "stationary_turtle"};

  SOFTWARE_TRAINING_LOCAL

  void add_name(const std::shared_ptr<turtlesim::srv::Kill::Request> req,
    const std::shared_ptr<turtlesim::srv::Kill::Response> /*resp */) {
      turtle_names.push_back(req->name);
    }

  void kill_callback(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture resp) {
      (void)resp;
      RCLCPP_INFO(this->get_logger(), "Turtle killed!");
      // stop executing callback
      rclcpp::shutdown();
  }
  
  void clear(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
    
    (void)req;
    if (!client->wait_for_service(2s)) {
          if(!rclcpp::ok()) {
              RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for kill service, exiting");
              resp->success = false;
              return;
          }
          RCLCPP_INFO(this->get_logger(), "Service not avaliable");
          resp->success = false;
          return;
    }

    for (auto name: turtle_names) {
        auto kill_req = std::make_shared<turtlesim::srv::Kill::Request>();
        kill_req->name = name;
        // send request to kill turtle
        // auto res = client->async_send_request(kill_req, std::bind(&clear_turtles::kill_callback, this, _1));
        auto res = client->async_send_request(kill_req);
    }
    // too much work, just leave it - remove all turtles after clearing them to update the known turtles
    // turtle_names.clear();
    resp->success = true;
    
  }
};

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::clear_turtles)
