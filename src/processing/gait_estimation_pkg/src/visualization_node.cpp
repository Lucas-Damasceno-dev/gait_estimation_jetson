#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <interfaces/msg/synced_sensor_data.hpp>

#include <zmq.h>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

class VisualizationNode : public rclcpp::Node
{
public:
  VisualizationNode() : Node("visualization_node"), zmq_context_(nullptr), zmq_socket_(nullptr)
  {
    // Parâmetros de configuração
    this->declare_parameter<bool>("use_coppeliasim", true);
    this->declare_parameter<std::string>("coppeliasim_host", "127.0.0.1");
    this->declare_parameter<int>("coppeliasim_port", 23000);
    this->declare_parameter<std::vector<std::string>>(
      "joint_names", 
      {"hip_joint_r", "knee_joint_r", "ankle_joint_r", "hip_joint_l", "knee_joint_l", "ankle_joint_l"}
    );

    use_coppeliasim_ = this->get_parameter("use_coppeliasim").as_bool();
    coppeliasim_host_ = this->get_parameter("coppeliasim_host").as_string();
    coppeliasim_port_ = this->get_parameter("coppeliasim_port").as_int();
    joint_names_ = this->get_parameter("joint_names").as_string_array();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    // Inscrição para receber dados de articulações e marcha estimada
    sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/gait/joint_states", qos,
      std::bind(&VisualizationNode::joint_state_callback, this, std::placeholders::_1)
    );

    // Publicador RViz2 nativo (se receber de outro nó ou republicar)
    pub_rviz_joints_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);

    // Inicializa comunicação ZeroMQ com CoppeliaSim
    if (use_coppeliasim_) {
      init_zmq();
    }

    RCLCPP_INFO(this->get_logger(), "VisualizationNode (C++) pronto. CoppeliaSim ZeroMQ: %s", 
                use_coppeliasim_ ? "ATIVADO" : "DESATIVADO");
  }

  virtual ~VisualizationNode()
  {
    if (zmq_socket_) {
      zmq_close(zmq_socket_);
    }
    if (zmq_context_) {
      zmq_ctx_destroy(zmq_context_);
    }
  }

private:
  void init_zmq()
  {
    zmq_context_ = zmq_ctx_new();
    zmq_socket_ = zmq_socket(zmq_context_, ZMQ_PUB);

    std::string endpoint = "tcp://" + coppeliasim_host_ + ":" + std::to_string(coppeliasim_port_);
    int linger = 0;
    zmq_setsockopt(zmq_socket_, ZMQ_LINGER, &linger, sizeof(linger));

    int rc = zmq_bind(zmq_socket_, endpoint.c_str());
    if (rc != 0) {
      // Se bind falhar (ex: porta ocupada ou simulador já em listen), tenta connect
      rc = zmq_connect(zmq_socket_, endpoint.c_str());
    }

    if (rc == 0) {
      RCLCPP_INFO(this->get_logger(), "ZeroMQ Publisher conectado com sucesso em: %s", endpoint.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Aviso: Falha ao iniciar ZeroMQ no endpoint %s (errno: %d)", endpoint.c_str(), errno);
    }
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // 1. Republica para o RViz2 no tópico /joint_states
    pub_rviz_joints_->publish(*msg);

    // 2. Transmite frame compacto JSON/CSV via ZeroMQ para o modelo antropomórfico no CoppeliaSim
    if (use_coppeliasim_ && zmq_socket_) {
      send_to_coppeliasim(msg);
    }
  }

  void send_to_coppeliasim(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Formata mensagem compacta para a API ZeroMQ do CoppeliaSim
    // Exemplo: {"hip": 0.12, "knee": -0.45, "ankle": 0.05}
    std::ostringstream ss;
    ss << "{";
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      ss << "\"" << msg->name[i] << "\":" << std::fixed << std::setprecision(4) << msg->position[i];
      if (i + 1 < msg->name.size()) {
        ss << ",";
      }
    }
    ss << "}";

    std::string payload = ss.str();
    zmq_send(zmq_socket_, payload.c_str(), payload.length(), ZMQ_DONTWAIT);
  }

  bool use_coppeliasim_;
  std::string coppeliasim_host_;
  int coppeliasim_port_;
  std::vector<std::string> joint_names_;

  void* zmq_context_;
  void* zmq_socket_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_rviz_joints_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
