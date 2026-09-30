#include <rclcpp/rclcpp.hpp>
#include <interfaces/msg/pressure_state.hpp>
#include <iostream>
#include <string>
#include <algorithm>

class MonitorCalcanharNode : public rclcpp::Node
{
public:
  MonitorCalcanharNode() : Node("monitor_calcanhar_node")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    sub_ = this->create_subscription<interfaces::msg::PressureState>(
      "/pressure/state", qos,
      std::bind(&MonitorCalcanharNode::callback, this, std::placeholders::_1)
    );

    std::cout << "\n=================================================================\n";
    std::cout << " 🦶 MONITOR DE CALCANHAR NATIVO C++ (Tópico: /pressure/state)\n";
    std::cout << " Aguardando pacotes das palmilhas... Pressione o calcanhar!\n";
    std::cout << "=================================================================\n\n";
  }

private:
  void callback(const interfaces::msg::PressureState::SharedPtr msg)
  {
    for (size_t idx = 0; idx < msg->pressures.size(); ++idx) {
      std::string nome = (idx < msg->names.size()) ? msg->names[idx] : ("Palmilha_" + std::to_string(idx));
      const auto& p_data = msg->pressures[idx];

      int c1 = (p_data.pressures.size() > 0) ? p_data.pressures[0].pressure : 0;
      int c2 = (p_data.pressures.size() > 1) ? p_data.pressures[1].pressure : 0;
      int c3 = (p_data.pressures.size() > 2) ? p_data.pressures[2].pressure : 0;
      int c4 = (p_data.pressures.size() > 3) ? p_data.pressures[3].pressure : 0;
      int total = c1 + c2 + c3 + c4;

      int bar_len = std::min(15, total / 350);
      std::string barra(bar_len, '#');
      barra.append(15 - bar_len, '-');

      printf("[%s] Calcanhar: S1=%4d | S2=%4d | S3=%4d | S4=%4d | Total=%5d | [%s]\n",
             nome.c_str(), c1, c2, c3, c4, total, barra.c_str());
    }
  }

  rclcpp::Subscription<interfaces::msg::PressureState>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitorCalcanharNode>());
  rclcpp::shutdown();
  return 0;
}
