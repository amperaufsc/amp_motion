#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>





#include "fs_msgs/msg/control_command.hpp"
#include <chrono>

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FloatPublisherLifecycle : public rclcpp_lifecycle::LifecycleNode
{
public:
  FloatPublisherLifecycle() : rclcpp_lifecycle::LifecycleNode("check_lifecyle_node")


  {
    // No padrão Lifecycle, o construtor apenas inicializa variáveis nativas.
    // Nada que envolva o ROS 2 (timers, publishers) deve ser criado aq.
    msg.steering = 0.0;
    msg.throttle = 734.0;

    RCLCPP_INFO(this->get_logger(), "Nó Lifecycle inicializado.");
  }

  // --- Funções de Transição de Estado do Lifecycle ---

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Configurando o nó...");

    // Cria o publisher. Ele nasce "dormindo" (Unconfigured -> Inactive)

    publisher_ = this->create_publisher<fs_msgs::msg::ControlCommand>("/control_command", 10);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Ativando o nó e iniciando o controle...");

    // 1. Ativa explicitamente o publicador do Lifecycle
    publisher_->on_activate();

    // 2. Cria o timer apenas agora, para poupar processamento enquanto estiver inativo
    timer_ = this->create_wall_timer(
      500ms, std::bind(&FloatPublisherLifecycle::timer_callback, this));

    return rclcpp_lifecycle::LifecycleNode::on_activate(state);
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Desativando o nó e pausando o controle...");
    
    // 1. Desativa o publicador
    publisher_->on_deactivate();

    // 2. Destrói o timer para que o callback pare de rodar imediatamente
    timer_.reset();

    return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Limpando o nó da memória...");
    
    // Destrói o publicador
    publisher_.reset();
    
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Encerrando nó Lifecycle...");
    timer_.reset();
    publisher_.reset();
    
    return CallbackReturn::SUCCESS;
  }

private:
  void timer_callback()
  {



    RCLCPP_INFO(this->get_logger(), "Publicando throttle: %f", msg.throttle);
    RCLCPP_INFO(this->get_logger(), "Publicando steering: %f", msg.steering);

    // Opcional, mas seguro: Só publica se o LifecyclePublisher realmente estiver ativo
    if (publisher_->is_activated()) {
      publisher_->publish(msg);
    }

    msg.steering += variacao_steering;
    msg.throttle += variacao_throttle;

    if (msg.steering <= -1.0f || msg.steering >= 1.0f) {
      variacao_steering = -variacao_steering;
    }

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

  // Atenção à mudança de tipo: Agora é um LifecyclePublisher
  rclcpp_lifecycle::LifecyclePublisher<fs_msgs::msg::ControlCommand>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  int count = 0;
  fs_msgs::msg::ControlCommand msg;
  float variacao_steering = 1.0;
  float variacao_throttle = 50.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Para compilar um nó Lifecycle no rclcpp::spin, você deve extrair a interface base dele
  auto node = std::make_shared<FloatPublisherLifecycle>();
  rclcpp::spin(node->get_node_base_interface());
  
  rclcpp::shutdown();
  return 0;
}