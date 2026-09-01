#include <rclcpp/rclcpp.hpp>
#include <interfaces/msg/pressure_state.hpp>
#include <interfaces/msg/pressure_data.hpp>
#include <interfaces/msg/pressure_unit_sensor.hpp>
#include <interfaces/msg/imu_state.hpp>
#include <interfaces/msg/imu_data.hpp>
#include <common_serial/serial_handler.hpp>
#include <common_serial/stm32_protocol.hpp>

#include <vector>
#include <string>
#include <sstream>
#include <memory>

class SerialDriverNode : public rclcpp::Node
{
public:
  SerialDriverNode() : Node("serial_driver_node")
  {
    // Declaração de parâmetros
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baudrate", 115200);
    this->declare_parameter<std::string>("device_name", "left_insole");
    this->declare_parameter<int>("timer_period_ms", 2); // Leitura em ~500Hz

    port_ = this->get_parameter("port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    device_name_ = this->get_parameter("device_name").as_string();
    int timer_ms = this->get_parameter("timer_period_ms").as_int();

    // Configura QoS de alto desempenho
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    pub_pressure_ = this->create_publisher<interfaces::msg::PressureState>("pressure/state", qos);
    pub_imu_ = this->create_publisher<interfaces::msg::IMUState>("imu/state", qos);

    // Inicialização da porta serial
    serial_ = std::make_unique<SerialHandler>();
    if (serial_->init(port_.c_str()) < 0) {
      RCLCPP_WARN(this->get_logger(), "Aviso: Não foi possível abrir porta %s (Hardware desconectado). Aguardando conexão...", port_.c_str());
    } else {
      serial_->setDefaultConfig();
      serial_->setBaudRate(baudrate_);
      RCLCPP_INFO(this->get_logger(), "Porta serial %s aberta com sucesso a %d bps.", port_.c_str(), baudrate_);
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_ms),
      std::bind(&SerialDriverNode::read_serial_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "SerialDriverNode pronto para telemetria STM32 (Palmilha 16-ch + IMUs).");
  }

private:
  void read_serial_callback()
  {
    uint8_t buffer[512];
    ssize_t bytes_read = serial_->readData(buffer, sizeof(buffer));

    if (bytes_read <= 0) {
      return;
    }

    rx_buffer_.insert(rx_buffer_.end(), buffer, buffer + bytes_read);

    // 1. Tenta processar pacotes binários estruturados
    process_binary_frames();

    // 2. Fallback para strings ASCII (caso o STM32 esteja enviando dados legados formatados em texto)
    process_ascii_lines();

    // Previne overflow do buffer em caso de ruído excessivo
    if (rx_buffer_.size() > 2048) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 1024);
    }
  }

  void process_binary_frames()
  {
    constexpr size_t FRAME_SIZE = sizeof(STM32SensorFrame);

    while (rx_buffer_.size() >= FRAME_SIZE) {
      // Procura pelo Start of Frame (0xAA 0x55)
      if (rx_buffer_[0] == STM32_SOF1 && rx_buffer_[1] == STM32_SOF2) {
        STM32SensorFrame frame;
        std::memcpy(&frame, rx_buffer_.data(), FRAME_SIZE);

        // Valida Checksum CRC16
        uint16_t expected_crc = calculate_checksum16(
          rx_buffer_.data(),
          FRAME_SIZE - sizeof(uint16_t)
        );

        if (frame.checksum == expected_crc) {
          publish_telemetry(frame);
          rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + FRAME_SIZE);
          continue;
        } else {
          // Checksum inválido: avança 1 byte para sincronizar novamente
          rx_buffer_.erase(rx_buffer_.begin());
        }
      } else {
        // Não é SOF: procura o próximo byte
        break;
      }
    }
  }

  void process_ascii_lines()
  {
    std::string text(rx_buffer_.begin(), rx_buffer_.end());
    size_t pos = text.find('\n');

    while (pos != std::string::npos) {
      std::string line = text.substr(0, pos);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }

      std::stringstream ss(line);
      std::string token;
      std::vector<float> values;

      while (ss >> token) {
        try {
          values.push_back(std::stof(token));
        } catch (...) {
          // Ignora caracteres não numéricos
        }
      }

      if (values.size() >= 16) {
        publish_ascii_telemetry(values);
      }

      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + pos + 1);
      text = std::string(rx_buffer_.begin(), rx_buffer_.end());
      pos = text.find('\n');
    }
  }

  void publish_telemetry(const STM32SensorFrame& frame)
  {
    auto now = this->now();

    // 1. Mensagem de Pressão
    interfaces::msg::PressureState press_msg;
    press_msg.header.stamp = now;
    press_msg.header.frame_id = device_name_;
    press_msg.names.push_back(device_name_);

    interfaces::msg::PressureData p_data;
    for (size_t i = 0; i < 16; ++i) {
      interfaces::msg::PressureUnitSensor unit;
      unit.id = static_cast<uint8_t>(i + 1);
      unit.pressure = static_cast<int16_t>(frame.pressure_channels[i]);
      p_data.pressures.push_back(unit);
    }
    press_msg.pressures.push_back(p_data);
    pub_pressure_->publish(press_msg);

    // 2. Mensagem de IMU
    interfaces::msg::IMUState imu_msg;
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = "imu_link";

    uint8_t valid_imus = (frame.num_imus > 3) ? 3 : frame.num_imus;
    for (uint8_t i = 0; i < valid_imus; ++i) {
      interfaces::msg::IMUData imu_data;
      imu_data.name = "imu_" + std::to_string(i + 1);
      imu_data.q_w = frame.imus[i].q_w;
      imu_data.q_x = frame.imus[i].q_x;
      imu_data.q_y = frame.imus[i].q_y;
      imu_data.q_z = frame.imus[i].q_z;
      imu_msg.imus.push_back(imu_data);
    }
    pub_imu_->publish(imu_msg);
  }

  void publish_ascii_telemetry(const std::vector<float>& values)
  {
    auto now = this->now();

    interfaces::msg::PressureState press_msg;
    press_msg.header.stamp = now;
    press_msg.header.frame_id = device_name_;
    press_msg.names.push_back(device_name_);

    interfaces::msg::PressureData p_data;
    for (size_t i = 0; i < 16 && i < values.size(); ++i) {
      interfaces::msg::PressureUnitSensor unit;
      unit.id = static_cast<uint8_t>(i + 1);
      unit.pressure = static_cast<int16_t>(values[i]);
      p_data.pressures.push_back(unit);
    }
    press_msg.pressures.push_back(p_data);
    pub_pressure_->publish(press_msg);
  }

  std::string port_;
  int baudrate_;
  std::string device_name_;
  std::vector<uint8_t> rx_buffer_;

  std::unique_ptr<SerialHandler> serial_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<interfaces::msg::PressureState>::SharedPtr pub_pressure_;
  rclcpp::Publisher<interfaces::msg::IMUState>::SharedPtr pub_imu_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialDriverNode>());
  rclcpp::shutdown();
  return 0;
}
