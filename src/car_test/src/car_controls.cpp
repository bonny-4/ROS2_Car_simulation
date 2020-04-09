#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace std::chrono_literals;

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("car_controls");
  auto publisher = node->create_publisher<std_msgs::msg::String>("car_controls_topic", 10);
  std_msgs::msg::String message;
  rclcpp::WallRate loop_rate(250ms);
  char input;

  cout << "Select your option : z(Throttle), s(Brake) or else(to do nothing)";
  while (rclcpp::ok())
  {
    cin>>input;
    switch(input)
    {
        case 'z':
                message.data = "Throttle";
                break;
        case 's':
                message.data = "Brake";
                break;
        default:
                message.data = "";
    }
    
    RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
