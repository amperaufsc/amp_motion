#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
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
    msg.header.frame_id = ""; 
    msg.steering = 0.0f;
    msg.throttle = 734.0f;
    msg.brake = 0.0f; // Inicialize o brake também, já que ele existe no .msg!

  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Configurando: Criando o lifecycle publisher...");
    
    // SOLUÇÃO DO ERRO: No Humble, usamos o "create_publisher" padrão.
    // Como a classe herda de LifecycleNode, ele gera automaticamente o tipo LifecyclePublisher.
    publisher_ = this->create_publisher<fs_msgs::msg::ControlCommand>("/control_command", 10);
    
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Ativando: Ativando publisher e iniciando timer...");
    
    // Ativa a transmissão do publisher de ciclo de vida
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
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link"; 
    
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

  // Mantemos o tipo correto aqui para gerenciar os estados de on_activate()
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
