#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }

  // make node
  std::shared_ptr<rclcpp::Node> node = rcl::Node::make_shared("add_two_ints_client");
  // create client for the node
  rclcpp::Client<example_interfaces::srv::AddTwoInts::Request>::SharedPtr client = node->create_client<example_interfaces::srv:AddTwoInts>("add_two_ints");

  auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  req->a = atoll(argv[1]);
  req->b = atoll(argv[2]);


  rclcpp::shutdown();
  return 0;

}