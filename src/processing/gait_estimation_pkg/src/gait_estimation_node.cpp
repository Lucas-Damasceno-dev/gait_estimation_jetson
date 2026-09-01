#include <rclcpp/rclcpp.hpp>
#include <interfaces/msg/synced_sensor_data.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

#include "gait_estimation_pkg/gait_features.hpp"
#include "gait_estimation_pkg/gait_models.hpp"

#include <chrono>

class GaitEstimationNode : public rclcpp::Node
{
public:
  GaitEstimationNode() : Node("gait_estimation_node"), last_time_(this->now())
  {
    // Parâmetros configuráveis
    this->declare_parameter<std::string>("model_type", "random_forest"); // "svm" ou "random_forest"
    model_type_ = this->get_parameter("model_type").as_string();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    // Inscrição no fluxo síncrono de dados do STM32 (/sync/data)
    sub_synced_ = this->create_subscription<interfaces::msg::SyncedSensorData>(
      "/sync/data", qos,
      std::bind(&GaitEstimationNode::synced_data_callback, this, std::placeholders::_1)
    );

    // Publicadores
    pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("/gait/joint_states", qos);
    pub_gait_phase_ = this->create_publisher<std_msgs::msg::String>("/gait/phase", qos);
    pub_inference_latency_ = this->create_publisher<std_msgs::msg::Float64>("/gait/latency_ms", qos);

    RCLCPP_INFO(this->get_logger(), "GaitEstimationNode (C++ AI Engine) iniciado. Modelo selecionado: %s", model_type_.c_str());
  }

private:
  void synced_data_callback(const interfaces::msg::SyncedSensorData::SharedPtr msg)
  {
    auto start_time = std::chrono::high_resolution_clock::now();
    auto now = this->now();

    double dt = (now - last_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) dt = 0.01;
    last_time_ = now;

    // 1. Extração de atitude dos 3 IMUs (Pitch Coxa, Canela, Pé)
    float pitch_thigh = 0.0f;
    float pitch_shank = 0.0f;
    float pitch_foot = 0.0f;

    for (const auto& imu : msg->imu_data.imus) {
      double sinp = 2.0 * (imu.q_w * imu.q_y - imu.q_z * imu.q_x);
      float pitch = static_cast<float>((std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp));

      if (imu.name == "imu_thigh" || imu.name == "imu_1") pitch_thigh = pitch;
      else if (imu.name == "imu_shank" || imu.name == "imu_2") pitch_shank = pitch;
      else if (imu.name == "imu_foot" || imu.name == "imu_3") pitch_foot = pitch;
    }

    // 2. Extração de pressão plantar (16 canais da primeira palmilha disponível)
    std::vector<uint16_t> raw_pressures(16, 0);
    if (!msg->pressure_data.pressures.empty()) {
      const auto& p_data = msg->pressure_data.pressures[0];
      for (size_t i = 0; i < 16 && i < p_data.pressures.size(); ++i) {
        raw_pressures[i] = static_cast<uint16_t>(p_data.pressures[i].pressure);
      }
    }

    // 3. Extração do Vetor de Características Biomecânicas
    GaitFeatureVector features = extract_gait_features(
      raw_pressures,
      pitch_thigh,
      pitch_shank,
      pitch_foot,
      static_cast<float>(dt)
    );

    // 4. Execução da Inferência de Fase da Marcha
    GaitPhase predicted_phase;
    if (model_type_ == "svm") {
      predicted_phase = svm_classifier_.predict(features);
    } else {
      predicted_phase = rf_classifier_.predict(features);
    }

    // 5. Estimação Cinemática de Juntas
    EstimatedJointAngles angles = mlp_estimator_.predict(features);

    // 6. Medição de Latência de Inferência
    auto end_time = std::chrono::high_resolution_clock::now();
    double latency_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    // 7. Publicação dos Resultados
    // Tópico JointState
    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = now;
    joint_msg.header.frame_id = "human_skeleton";
    joint_msg.name = {"hip_joint_r", "knee_joint_r", "ankle_joint_r"};
    joint_msg.position = {angles.hip_angle_rad, angles.knee_angle_rad, angles.ankle_angle_rad};
    pub_joint_states_->publish(joint_msg);

    // Tópico Fase da Marcha
    std_msgs::msg::String phase_msg;
    phase_msg.data = gait_phase_to_string(predicted_phase);
    pub_gait_phase_->publish(phase_msg);

    // Tópico de Latência (Garantia do requisito < 10ms)
    std_msgs::msg::Float64 lat_msg;
    lat_msg.data = latency_ms;
    pub_inference_latency_->publish(lat_msg);
  }

  std::string model_type_;
  rclcpp::Time last_time_;

  SVMGaitClassifier svm_classifier_;
  RandomForestGaitClassifier rf_classifier_;
  MLPKinematicEstimator mlp_estimator_;

  rclcpp::Subscription<interfaces::msg::SyncedSensorData>::SharedPtr sub_synced_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_gait_phase_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_inference_latency_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GaitEstimationNode>());
  rclcpp::shutdown();
  return 0;
}
