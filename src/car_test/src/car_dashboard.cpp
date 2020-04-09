#include <memory>
#include <iostream>
#include <iomanip>
#include <string.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "car_test_interfaces/srv/new_max_speed.hpp"

using std::placeholders::_1;
using NewMaxSpeed = car_test_interfaces::srv::NewMaxSpeed;

using namespace std;

int Current_Speed = 0;
int Current_MaxSpeed = 0;
string Current_control;

void Display_Dashboard (void)
{
    system("clear");
    cout << fixed << setprecision(2) << setfill('0');
    cout << "\\                  |                  ||                   |                 / " << endl;
    cout << " \\                 |                  ||                   |                / " << endl;
    cout << "  \\                |                  ||                   |               / " << endl;
    cout << "   \\               |                  ||                   |   -----      / " << endl;
    cout << "    \\              |                  ||                   |  /     \\    / " << endl;
    cout << "     \\             |                  ||                   |  \\ ";
    cout << setw(3) << Current_MaxSpeed << " /   / " << endl;
    cout << "      \\            |                  ||                   |   -----   / " << endl;
    cout << "       \\           |                  ||                   |     |    / " << endl;
    cout << "        \\          |                  ||                   |     |   / " << endl;
    cout << "         \\__________________________________________________________/ " << endl;
    cout << "         /                  ----------------------                   \\ " << endl;
    cout << "        /                   |                    |                    \\ " << endl;
    cout << "       /                    |       " << setw(3) << Current_Speed << " km/h     |                     \\ " << endl;
    cout << "      /                     |                    |                      \\ " << endl;
    cout << "     /                      ----------------------                       \\ " << endl;
    cout << endl;
    cout << "                                      Brake    Throttle" << endl;
    cout << "                                        |          |" << endl;
    cout << "                                        |          |" << endl;
    cout << "                                     -------    -------" << endl;
    cout << "                                     |     |    |     |" << endl;
    cout << "                                     |  ";
    if(Current_control == "Brake")
        cout << "X";
    else
        cout << " ";
    cout << "  |    |  ";
    if(Current_control == "Throttle")
        cout << "X";
    else
        cout << " ";
    cout << "  |" << endl;
    cout << "                                     |     |    |     |" << endl;
    cout << "                                     -------    -------" << endl;
}

// Classe gerant les topics
class DashBoard_topics : public rclcpp::Node
{
public:
  DashBoard_topics(): Node("car_dashboard")
  {
    subscription_current_speed = this->create_subscription<std_msgs::msg::Int32>("current_speed", 10, std::bind(&DashBoard_topics::current_speed_callback, this, _1));
    subscription_current_control = this->create_subscription<std_msgs::msg::String>("car_controls_topic", 10, std::bind(&DashBoard_topics::current_control_callback, this, _1));

    client_max_speed = this->create_client<car_test_interfaces::srv::NewMaxSpeed>("set_max_speed");
    timer_ = this->create_wall_timer(1s, std::bind(&DashBoard_topics::timer_callback, this));
  }
  
  void queue_async_request()
  {
    while (!client_max_speed->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    auto request = std::make_shared<car_test_interfaces::srv::NewMaxSpeed::Request>();
    request->newspeed = 0;

    // We give the async_send_request() method a callback that will get executed
    // once the response is received. This way we can return immediately from
    // this method and allow other work to be done by the executor in `spin`
    // while waiting for the response.
    using ServiceResponseFuture = rclcpp::Client<car_test_interfaces::srv::NewMaxSpeed>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future)
    {
      Current_MaxSpeed = future.get()->speedtaken;
    };
    auto future_result = client_max_speed->async_send_request(request, response_received_callback);
  }

private:
  // Callback for current speed
  void current_speed_callback(const std_msgs::msg::Int32::SharedPtr msg) const
  {
    //RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
    Current_Speed = msg->data;
    Display_Dashboard();
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_current_speed;
  
  // Callback for current control
  void current_control_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    //RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
    Current_control = msg->data.c_str();
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_current_control;

  // Callback for client service
  void timer_callback()
  {
    queue_async_request();
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<car_test_interfaces::srv::NewMaxSpeed>::SharedPtr client_max_speed;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DashBoard_topics>());
  rclcpp::shutdown();
  return 0;
}
