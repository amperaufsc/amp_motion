#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "fs_msgs/msg/control_command.hpp"

using namespace std::chrono_literals;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class check_lifecycle_node : public rclcpp_lifecycle::LifecycleNode
{
public:
  // 1. O construtor agora tem EXATAMENTE o mesmo nome da classe
  check_lifecycle_node()
  : LifecycleNode("check_lifecycle_node")
  {
    msg_.steering = 0.0;
    msg_.throttle = 734.0;
  }

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(get_logger(), "Configuring node...");

    publisher_ = this->create_publisher<fs_msgs::msg::ControlCommand>(
        "/control_command", 10);

    timer_ = this->create_wall_timer(
        500ms,
        // 2. Corrigido para referenciar a classe atual
        std::bind(&check_lifecycle_node::timer_callback, this));

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(get_logger(), "Activating node...");
    publisher_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating node...");
    publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(get_logger(), "Cleaning up...");

    timer_.reset();
    publisher_.reset();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(get_logger(), "Shutting down...");
    return CallbackReturn::SUCCESS;
  }

  void timer_callback()
  {
    // Publica apenas quando ACTIVE
    if (!publisher_->is_activated())
      return;

    RCLCPP_INFO(get_logger(), "Publishing throttle: %.2f", msg_.throttle);
    RCLCPP_INFO(get_logger(), "Publishing steering: %.2f", msg_.steering);

    publisher_->publish(msg_);

    msg_.steering += steering_step_;
    msg_.throttle += throttle_step_;

    if (msg_.steering <= -1.0f || msg_.steering >= 1.0f)
      steering_step_ = -steering_step_;

    if (msg_.throttle <= 734.0f || msg_.throttle >= 984.0f)
    {
      if (count_ <= 6)
      {
        throttle_step_ = -throttle_step_;
        count_++;
      }
      else
      {
        msg_.throttle = 0.0;
      }
    }

    RCLCPP_INFO(get_logger(), "-------------------------------");
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<fs_msgs::msg::ControlCommand>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  fs_msgs::msg::ControlCommand msg_;

  int count_ = 0;
  float steering_step_ = 1.0f;
  float throttle_step_ = 50.0f;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 3. Instanciando a classe correta no main
  auto node = std::make_shared<check_lifecycle_node>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}