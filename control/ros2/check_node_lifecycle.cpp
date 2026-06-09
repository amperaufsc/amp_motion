#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp" // Cabeçalho essencial para o escopo do publisher
#include "fs_msgs/msg/control_command.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FloatPublisherLifecycle : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit FloatPublisherLifecycle(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name, 
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
    msg.steering = 0.0f;
    msg.throttle = 734.0f;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Configurando: Criando o lifecycle publisher...");
    
    // RESOLVIDO: Chamada explícita pelo escopo da classe base para o ROS 2 Humble
    publisher_ = this->rclcpp_lifecycle::LifecycleNode::create_lifecycle_publisher<fs_msgs::msg::ControlCommand>(
      "/control_command", 10);
    
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Ativando: Ativando publisher e iniciando timer...");
    publisher_->on_activate();
    timer_ = this->create_wall_timer(500ms, std::bind(&FloatPublisherLifecycle::timer_callback, this));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Desativando: Desativando publisher e destruindo timer...");
    publisher_->on_deactivate();
    timer_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Limpando recursos: Resetando o publisher...");
    publisher_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Desligando o nó a partir do estado: %s", state.label().c_str());
    timer_.reset();
    publisher_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Publicando throttle: %f", msg.throttle);
    RCLCPP_INFO(this->get_logger(), "Publicando steering: %f", msg.steering);
    
    publisher_->publish(msg);
    
    msg.steering += variacao_steering;
    msg.throttle += variacao_throttle;
    
    if (msg.steering <= -1.0f || msg.steering >= 1.0f) variacao_steering = -variacao_steering;
    if ((msg.throttle <= 734.0f || msg.throttle >= 984.0f)) {
      if (count <= 6) {
        variacao_throttle = -variacao_throttle;
        count++;
      } else {
        msg.throttle = 0.0f;
      }
    }
    RCLCPP_INFO(this->get_logger(), "-------------------------------");
  }

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<fs_msgs::msg::ControlCommand>> publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count = 0;
  fs_msgs::msg::ControlCommand msg;
  float variacao_steering = 1.0f;
  float variacao_throttle = 50.0f;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto lc_node = std::make_shared<FloatPublisherLifecycle>("float_publisher");
  rclcpp::spin(lc_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

