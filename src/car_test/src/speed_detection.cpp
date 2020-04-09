#include "rclcpp/rclcpp.hpp"
#include "car_test_interfaces/srv/new_max_speed.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 2)
  {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: speed_detection MaxSpeed");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("speed_detection");
  rclcpp::Client<car_test_interfaces::srv::NewMaxSpeed>::SharedPtr client =
    node->create_client<car_test_interfaces::srv::NewMaxSpeed>("set_max_speed");

  auto request = std::make_shared<car_test_interfaces::srv::NewMaxSpeed::Request>();
  request->newspeed = atoll(argv[1]);

  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    if (result.get()->speedtaken == request->newspeed)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New Max Speed SUCCESSFULLY taken");
    else if (request->newspeed == 0)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current Max Speed : %d", result.get()->speedtaken);
    else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR New Max Speed not set");
  } 
  else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_max_speed");
  }

  rclcpp::shutdown();
  return 0;
}
