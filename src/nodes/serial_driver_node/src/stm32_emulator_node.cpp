#include <rclcpp/rclcpp.hpp>
#include <interfaces/msg/pressure_state.hpp>
#include <interfaces/msg/imu_state.hpp>
#include <common_serial/stm32_protocol.hpp>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class STM32EmulatorNode : public rclcpp::Node
{
public:
  STM32EmulatorNode() : Node("stm32_emulator_node"), sample_count_(0)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    pub_pressure_ = this->create_publisher<interfaces::msg::PressureState>("pressure/state", qos);
    pub_imu_ = this->create_publisher<interfaces::msg::IMUState>("imu/state", qos);

    // Emula geração a 100 Hz (~10 ms)
    timer_ = this->create_wall_timer(
      10ms,
      std::bind(&STM32EmulatorNode::generate_synthetic_telemetry, this)
    );

    RCLCPP_INFO(this->get_logger(), "STM32 Emulator Node ativo. Gerando dados de marcha sintéticos a 100Hz.");
  }

private:
  void generate_synthetic_telemetry()
  {
    auto now = this->now();
    double t = sample_count_ * 0.01;
    sample_count_++;

    // 1. Emula 2 Palmilhas (Left e Right) com 16 sensores cada
    interfaces::msg::PressureState press_msg;
    press_msg.header.stamp = now;
    press_msg.header.frame_id = "dual_insoles";
    press_msg.names = {"insole_left", "insole_right"};

    for (int foot = 0; foot < 2; ++foot) {
      interfaces::msg::PressureData p_data;
      double phase_offset = (foot == 0) ? 0.0 : 3.14159; // Defasagem de passo
      double contact = std::max(0.0, std::sin(2.0 * 3.14159 * 1.0 * t + phase_offset));

      for (int i = 0; i < 16; ++i) {
        interfaces::msg::PressureUnitSensor unit;
        unit.id = static_cast<uint8_t>(i + 1);
        // Calcanhar (0..3), Mediopé (4..7), Antepé (8..15)
        double scale = (i < 4) ? 2000.0 : ((i < 8) ? 800.0 : 1500.0);
        unit.pressure = static_cast<int16_t>(contact * scale + (rand() % 50));
        p_data.pressures.push_back(unit);
      }
      press_msg.pressures.push_back(p_data);
    }
    pub_pressure_->publish(press_msg);

    // 2. Emula 2 ou 3 IMUs (Coxa, Canela, Pé)
    interfaces::msg::IMUState imu_msg;
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = "lower_limb_imus";

    const std::vector<std::string> imu_names = {"imu_thigh", "imu_shank", "imu_foot"};
    for (size_t i = 0; i < 3; ++i) {
      interfaces::msg::IMUData imu;
      imu.name = imu_names[i];
      double angle = 0.5 * std::sin(2.0 * 3.14159 * 1.0 * t + i * 0.3);
      imu.q_w = std::cos(angle / 2.0);
      imu.q_x = 0.0;
      imu.q_y = std::sin(angle / 2.0);
      imu.q_z = 0.0;
      imu_msg.imus.push_back(imu);
    }
    pub_imu_->publish(imu_msg);
  }

  uint64_t sample_count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<interfaces::msg::PressureState>::SharedPtr pub_pressure_;
  rclcpp::Publisher<interfaces::msg::IMUState>::SharedPtr pub_imu_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<STM32EmulatorNode>());
  rclcpp::shutdown();
  return 0;
}
