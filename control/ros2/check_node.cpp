#include <rclcpp/rclcpp.hpp>
#include "fs_msgs/msg/control_command.hpp"
#include <chrono>

using namespace std::chrono_literals;

class FloatPublisher : public rclcpp::Node
{
public:
  FloatPublisher()  : Node("float_publisher")
  {
    msg.steering = -100.0;
    msg.throttle = 734.0;
    publisher_ = this->create_publisher<fs_msgs::msg::ControlCommand>("/control_command", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&FloatPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Publicando throttle: %f", msg.throttle);
    RCLCPP_INFO(this->get_logger(), "Publicando steering: %f", msg.steering);
    publisher_->publish(msg);
    if (steering_counter >= 10*3){
      msg.steering += variacao_steering;
      steering_counter = -1;
    };
    steering_counter++;
    msg.throttle += variacao_throttle;

    if (msg.steering <= -100.0f || msg.steering >= 100.0f) variacao_steering = -variacao_steering;
    if ((msg.throttle <= 734.0f || msg.throttle >= 984.0f)) {
      if (count <= 6) {
        variacao_throttle = -variacao_throttle;
        count++;
      } else {
        msg.throttle = 0;
      }
    }

    RCLCPP_INFO(this->get_logger(), "-------------------------------");
  }

  rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count = 0;
  int steering_counter = 0;
  fs_msgs::msg::ControlCommand msg;
  float variacao_steering = 200.0;
  float variacao_throttle = 50.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FloatPublisher>());
  rclcpp::shutdown();
  return 0;
}