#include <rclcpp/rclcpp.hpp>
#include <interfaces/msg/pressure_state.hpp>
#include <iostream>
#include <string>
#include <iomanip>

class MonitorPalmilhasCompletoNode : public rclcpp::Node
{
public:
  MonitorPalmilhasCompletoNode() : Node("monitor_palmilhas_completo_node")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    sub_ = this->create_subscription<interfaces::msg::PressureState>(
      "/pressure/state", qos,
      std::bind(&MonitorPalmilhasCompletoNode::callback, this, std::placeholders::_1)
    );

    std::cout << "\n=========================================================================================\n";
    std::cout << " 🦶 MONITOR COMPLETO DAS DUAS PALMILHAS (16 Sensores Cada) - ROS 2 C++\n";
    std::cout << " Tópico: /pressure/state | Portas: /dev/ttyACM0 (Left) e /dev/ttyACM1 (Right)\n";
    std::cout << "=========================================================================================\n\n";
  }

private:
  void callback(const interfaces::msg::PressureState::SharedPtr msg)
  {
    std::cout << "\033[2J\033[1;1H"; // Limpa tela e posiciona cursor no topo
    std::cout << "=== LEITURA DE PRESSÃO EM TEMPO REAL (" << msg->pressures.size() << " palmilha(s) detectada(s)) ===\n\n";

    for (size_t idx = 0; idx < msg->pressures.size(); ++idx) {
      std::string nome = (idx < msg->names.size()) ? msg->names[idx] : ("Insole_" + std::to_string(idx));
      const auto& p_data = msg->pressures[idx];

      int total_forca = 0;
      int heel_sum = 0;
      int midfoot_sum = 0;
      int forefoot_sum = 0;

      std::cout << "▶ " << nome << " (" << p_data.pressures.size() << " sensores ativos):\n";
      std::cout << "-----------------------------------------------------------------------------------------\n";

      // Exibe os 16 sensores organizados por região anatômica
      for (size_t i = 0; i < p_data.pressures.size(); ++i) {
        int val = p_data.pressures[i].pressure;
        total_forca += val;
        if (i < 4) heel_sum += val;
        else if (i < 8) midfoot_sum += val;
        else forefoot_sum += val;

        std::cout << " S" << std::setw(2) << std::setfill('0') << (i + 1) << ":" << std::setw(4) << std::setfill(' ') << val << " ";
        if ((i + 1) % 8 == 0) std::cout << "\n";
      }

      std::cout << "\n Regiões Anatômicas:\n";
      std::cout << "  • Calcanhar (S01-S04): " << std::setw(5) << heel_sum << " N\n";
      std::cout << "  • Mediopé   (S05-S08): " << std::setw(5) << midfoot_sum << " N\n";
      std::cout << "  • Antepé    (S09-S16): " << std::setw(5) << forefoot_sum << " N\n";
      std::cout << "  • Carga Total Palmilha: " << std::setw(5) << total_forca << " N\n";
      std::cout << "-----------------------------------------------------------------------------------------\n\n";
    }
  }

  rclcpp::Subscription<interfaces::msg::PressureState>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitorPalmilhasCompletoNode>());
  rclcpp::shutdown();
  return 0;
}
