#include <chrono>
#include <memory>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "car_test_interfaces/srv/new_max_speed.hpp"

using namespace std::chrono_literals;

using NewMaxSpeed = car_test_interfaces::srv::NewMaxSpeed;
rclcpp::Node::SharedPtr setmaxspeed_node = nullptr;

using std::placeholders::_1;

int Max_Speed = 30;
int Current_Speed = 0;
int Delta_Speed = 0;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
// Classe gerant la publication du topic de la vitesse courante
class Speed_regulation : public rclcpp::Node
{
  public:
  Speed_regulation() : Node("Speed_regulation")
  {
    // Topics
    // Topic to write car speed
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("current_speed", 10);
    timer_ = this->create_wall_timer(250ms, std::bind(&Speed_regulation::timer_callback, this));
    // Topic to read car controls
    subscription_ = this->create_subscription<std_msgs::msg::String>("car_controls_topic", 10, std::bind(&Speed_regulation::topic_callback, this, _1));
  }

  private:
  // Timer callback to send topic
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    
    Current_Speed += Delta_Speed;

    if (Current_Speed <= 0 || Max_Speed == 0)
        Current_Speed = 0;
    if (Current_Speed >= Max_Speed)
        Current_Speed = Max_Speed;

    message.data = Current_Speed;
    RCLCPP_INFO(this->get_logger(), "Current_Speed: '%d'", message.data);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

  // Callback when topic is received
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    if(strcmp(msg->data.c_str(), "Throttle") == 0)
        Delta_Speed = 5;
    else if(strcmp(msg->data.c_str(), "Brake") == 0)
        Delta_Speed = -5;
    else
        Delta_Speed = -1;
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

// Clase gerant le service de reception de la vitesse maximale
class Set_MaxSpeed_Service : public rclcpp::Node
{
  public:  Set_MaxSpeed_Service() : Node("set_max_speed_service")
  {    
   auto handle_set_maxspeed = [this](            
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<NewMaxSpeed::Request> request,
    std::shared_ptr<NewMaxSpeed::Response> response) -> 
    void {      
      (void)request_header;
      if (request->newspeed != 0)
        Max_Speed = request->newspeed;
      response->speedtaken = Max_Speed;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nMax Speed %d", request->newspeed);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", response->speedtaken);
    };     
   srv_ = this->create_service<NewMaxSpeed>("set_max_speed", handle_set_maxspeed);  
  } 
  private:  
  rclcpp::Service<NewMaxSpeed>::SharedPtr srv_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto service_node = std::make_shared<Set_MaxSpeed_Service>();
  auto topic_node = std::make_shared<Speed_regulation>();

  executor.add_node(service_node);
  executor.add_node(topic_node);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
