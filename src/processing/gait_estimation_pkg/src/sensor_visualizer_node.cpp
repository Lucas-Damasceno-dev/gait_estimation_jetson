#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32.hpp>
#include <interfaces/msg/pressure_state.hpp>
#include <interfaces/msg/imu_state.hpp>

#include <chrono>
#include <vector>
#include <string>
#include <deque>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <algorithm>

using namespace std::chrono_literals;

struct RateEstimator
{
  std::deque<std::chrono::steady_clock::time_point> window;
  std::chrono::steady_clock::time_point last_rx_time;
  double hz{0.0};
  double dt_ms{0.0};
  uint64_t total_packets{0};
  bool received_first{false};

  void on_packet()
  {
    auto now = std::chrono::steady_clock::now();
    total_packets++;
    if (received_first) {
      dt_ms = std::chrono::duration<double, std::milli>(now - last_rx_time).count();
    }
    last_rx_time = now;
    received_first = true;
    window.push_back(now);

    while (!window.empty() &&
           std::chrono::duration<double>(now - window.front()).count() > 1.0) {
      window.pop_front();
    }

    if (window.size() >= 2) {
      double span = std::chrono::duration<double>(window.back() - window.front()).count();
      if (span > 0.001) {
        hz = static_cast<double>(window.size() - 1) / span;
      }
    }
  }

  void update_timeout(double timeout_sec = 1.0)
  {
    auto now = std::chrono::steady_clock::now();
    if (!received_first || std::chrono::duration<double>(now - last_rx_time).count() > timeout_sec) {
      hz = 0.0;
      dt_ms = 0.0;
    }
  }

  bool is_active() const
  {
    return hz > 0.1;
  }
};

class SensorVisualizerNode : public rclcpp::Node
{
public:
  SensorVisualizerNode()
  : Node("sensor_visualizer_node"),
    start_time_(std::chrono::steady_clock::now())
  {
    this->declare_parameter<std::string>("frame_id", "world");
    this->declare_parameter<double>("publish_rate_hz", 25.0);
    this->declare_parameter<double>("max_pressure_value", 3500.0);

    frame_id_ = this->get_parameter("frame_id").as_string();
    double rate_hz = this->get_parameter("publish_rate_hz").as_double();
    max_pressure_ = this->get_parameter("max_pressure_value").as_double();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    sub_pressure_ = this->create_subscription<interfaces::msg::PressureState>(
      "/pressure/state", qos,
      std::bind(&SensorVisualizerNode::pressure_callback, this, std::placeholders::_1));

    sub_imu_ = this->create_subscription<interfaces::msg::IMUState>(
      "/imu/state", qos,
      std::bind(&SensorVisualizerNode::imu_callback, this, std::placeholders::_1));

    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/sensor_monitor/markers", 10);

    pub_pressure_hz_ = this->create_publisher<std_msgs::msg::Float32>(
      "/sensor_monitor/pressure_hz", 10);
    pub_imu_hz_ = this->create_publisher<std_msgs::msg::Float32>(
      "/sensor_monitor/imu_hz", 10);

    init_geometry();

    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz));
    timer_ = this->create_wall_timer(
      timer_period, std::bind(&SensorVisualizerNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "SensorVisualizerNode iniciado. Publicando marcadores RViz em /sensor_monitor/markers");
  }

private:
  void init_geometry()
  {
    // Silhueta anatômica aproximada do pé (comprimento ~0.26m)
    left_foot_contour_ = {
      {-0.085f,  0.000f}, {-0.080f, -0.022f}, {-0.065f, -0.033f}, {-0.040f, -0.032f},
      {-0.010f, -0.025f}, { 0.020f, -0.022f}, { 0.050f, -0.035f}, { 0.080f, -0.042f},
      { 0.110f, -0.035f}, { 0.130f, -0.015f}, { 0.135f,  0.010f}, { 0.125f,  0.030f},
      { 0.090f,  0.042f}, { 0.060f,  0.040f}, { 0.020f,  0.035f}, {-0.020f,  0.030f},
      {-0.060f,  0.030f}, {-0.080f,  0.020f}, {-0.085f,  0.000f}
    };

    // Posições dos 16 sensores Legact FS-INS-16Z no pé esquerdo
    left_sensor_coords_ = {
      // 0..3: Calcanhar
      {-0.065f, -0.015f}, {-0.065f,  0.015f}, {-0.040f, -0.018f}, {-0.040f,  0.018f},
      // 4..7: Mediopé
      {-0.010f, -0.015f}, {-0.010f,  0.018f}, { 0.020f, -0.020f}, { 0.020f,  0.015f},
      // 8..15: Antepé / Dedos
      { 0.050f, -0.028f}, { 0.050f, -0.010f}, { 0.050f,  0.010f}, { 0.050f,  0.028f},
      { 0.080f, -0.025f}, { 0.080f, -0.005f}, { 0.080f,  0.018f}, { 0.105f, -0.010f}
    };
  }

  void pressure_callback(const interfaces::msg::PressureState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    rate_pressure_.on_packet();
    last_pressure_msg_ = msg;
  }

  void imu_callback(const interfaces::msg::IMUState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    rate_imu_.on_packet();
    last_imu_msg_ = msg;
  }

  std_msgs::msg::ColorRGBA get_jet_color(float val, float alpha = 0.9f)
  {
    val = std::clamp(val, 0.0f, 1.0f);
    std_msgs::msg::ColorRGBA c;
    c.a = alpha;
    if (val < 0.25f) {
      c.r = 0.0f;
      c.g = 4.0f * val;
      c.b = 1.0f;
    } else if (val < 0.5f) {
      c.r = 0.0f;
      c.g = 1.0f;
      c.b = 1.0f - 4.0f * (val - 0.25f);
    } else if (val < 0.75f) {
      c.r = 4.0f * (val - 0.5f);
      c.g = 1.0f;
      c.b = 0.0f;
    } else {
      c.r = 1.0f;
      c.g = 1.0f - 4.0f * (val - 0.75f);
      c.b = 0.0f;
    }
    return c;
  }

  geometry_msgs::msg::Point rotate_vector_by_quaternion(
    double vx, double vy, double vz,
    double qw, double qx, double qy, double qz)
  {
    // v' = v + 2*q_xyz x (q_xyz x v + qw*v)
    double tx = 2.0 * (qy * vz - qz * vy);
    double ty = 2.0 * (qz * vx - qx * vz);
    double tz = 2.0 * (qx * vy - qy * vx);

    geometry_msgs::msg::Point p;
    p.x = vx + qw * tx + (qy * tz - qz * ty);
    p.y = vy + qw * ty + (qz * tx - qx * tz);
    p.z = vz + qw * tz + (qx * ty - qy * tx);
    return p;
  }

  void timer_callback()
  {
    interfaces::msg::PressureState::SharedPtr p_msg;
    interfaces::msg::IMUState::SharedPtr imu_msg;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      rate_pressure_.update_timeout(1.2);
      rate_imu_.update_timeout(1.2);
      p_msg = last_pressure_msg_;
      imu_msg = last_imu_msg_;
    }

    // Publica valores numéricos de frequência
    std_msgs::msg::Float32 hz_p, hz_imu;
    hz_p.data = static_cast<float>(rate_pressure_.hz);
    hz_imu.data = static_cast<float>(rate_imu_.hz);
    pub_pressure_hz_->publish(hz_p);
    pub_imu_hz_->publish(hz_imu);

    // Constrói MarkerArray completo
    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;
    auto stamp = this->now();

    // 1. HUD / Banner Superior de Frequência em Hz
    build_hud_markers(markers, marker_id, stamp);

    // 2. Palmilhas 3D e Mapa de Pressão
    build_insole_markers(markers, marker_id, stamp, p_msg);

    // 3. IMUs 3D com Orientação e Eixos
    build_imu_markers(markers, marker_id, stamp, imu_msg);

    pub_markers_->publish(markers);
  }

  void build_hud_markers(
    visualization_msgs::msg::MarkerArray & markers,
    int & marker_id,
    const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::Marker hud;
    hud.header.frame_id = frame_id_;
    hud.header.stamp = stamp;
    hud.ns = "telemetry_hud";
    hud.id = marker_id++;
    hud.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    hud.action = visualization_msgs::msg::Marker::ADD;

    hud.pose.position.x = 0.0;
    hud.pose.position.y = 0.0;
    hud.pose.position.z = 0.55;
    hud.pose.orientation.w = 1.0;

    hud.scale.z = 0.042; // Altura da fonte

    auto uptime_sec = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::steady_clock::now() - start_time_).count();
    int min = uptime_sec / 60;
    int sec = uptime_sec % 60;

    std::ostringstream ss;
    ss << "╔═══════════════════════════════════════════════════════════╗\n";
    ss << "║        MONITOR DE TELEMETRIA EM TEMPO REAL (ROS 2)        ║\n";
    ss << "╠═══════════════════════════════════════════════════════════╣\n";
    ss << "║ 🦶 PALMILHAS : " << std::fixed << std::setprecision(1) << std::setw(5) << rate_pressure_.hz
       << " Hz  (dt: " << std::setw(5) << std::setprecision(1) << rate_pressure_.dt_ms << " ms)  "
       << (rate_pressure_.is_active() ? "● ATIVO [OK]" : "✖ TIMEOUT") << "     ║\n";
    ss << "║ 🧭 IMU (I2C) : " << std::fixed << std::setprecision(1) << std::setw(5) << rate_imu_.hz
       << " Hz  (dt: " << std::setw(5) << std::setprecision(1) << rate_imu_.dt_ms << " ms)  "
       << (rate_imu_.is_active() ? "● ATIVO [OK]" : "✖ TIMEOUT") << "     ║\n";
    ss << "║ ⏱️  UPTIME    : " << std::setfill('0') << std::setw(2) << min << ":"
       << std::setw(2) << sec << " | Total Pkts: P=" << rate_pressure_.total_packets
       << " I=" << rate_imu_.total_packets << "        ║\n";
    ss << "╚═══════════════════════════════════════════════════════════╝";

    hud.text = ss.str();

    if (rate_pressure_.is_active() && rate_imu_.is_active()) {
      hud.color.r = 0.1f; hud.color.g = 1.0f; hud.color.b = 0.2f; hud.color.a = 1.0f;
    } else if (rate_pressure_.is_active() || rate_imu_.is_active()) {
      hud.color.r = 1.0f; hud.color.g = 0.9f; hud.color.b = 0.1f; hud.color.a = 1.0f;
    } else {
      hud.color.r = 1.0f; hud.color.g = 0.2f; hud.color.b = 0.2f; hud.color.a = 1.0f;
    }

    markers.markers.push_back(hud);
  }

  void build_insole_markers(
    visualization_msgs::msg::MarkerArray & markers,
    int & marker_id,
    const rclcpp::Time & stamp,
    const interfaces::msg::PressureState::SharedPtr & p_msg)
  {
    constexpr float Y_OFFSET = 0.15f; // Esquerda +Y, Direita -Y

    // Renderiza ambos os pés (0 = Esquerdo, 1 = Direito)
    for (int foot = 0; foot < 2; ++foot) {
      bool is_left = (foot == 0);
      float y_base = is_left ? Y_OFFSET : -Y_OFFSET;
      float y_mirror = is_left ? 1.0f : -1.0f;
      std::string foot_name = is_left ? "PÉ ESQUERDO" : "PÉ DIREITO";

      // 1. Contorno Anatômico do Pé (LINE_STRIP)
      visualization_msgs::msg::Marker contour;
      contour.header.frame_id = frame_id_;
      contour.header.stamp = stamp;
      contour.ns = "insole_contour";
      contour.id = marker_id++;
      contour.type = visualization_msgs::msg::Marker::LINE_STRIP;
      contour.action = visualization_msgs::msg::Marker::ADD;
      contour.scale.x = 0.003; // Espessura da linha
      contour.color.r = 0.7f; contour.color.g = 0.7f; contour.color.b = 0.8f; contour.color.a = 0.9f;
      contour.pose.orientation.w = 1.0;

      for (const auto & pt : left_foot_contour_) {
        geometry_msgs::msg::Point p;
        p.x = pt.first;
        p.y = pt.second * y_mirror + y_base;
        p.z = 0.001;
        contour.points.push_back(p);
      }
      markers.markers.push_back(contour);

      // 2. Extrai leituras da palmilha correspondente
      std::vector<int16_t> sensor_values(16, 0);
      if (p_msg && foot < static_cast<int>(p_msg->pressures.size())) {
        const auto & insole_data = p_msg->pressures[foot].pressures;
        for (size_t s = 0; s < insole_data.size() && s < 16; ++s) {
          sensor_values[s] = insole_data[s].pressure;
        }
      }

      int total_force = 0;
      int heel_force = 0;
      int mid_force = 0;
      int fore_force = 0;
      double weighted_x = 0.0;
      double weighted_y = 0.0;

      // 3. Renderiza os 16 Sensores de Pressão
      for (size_t s = 0; s < 16; ++s) {
        float sx = left_sensor_coords_[s].first;
        float sy = left_sensor_coords_[s].second * y_mirror + y_base;
        int val = sensor_values[s];
        total_force += val;

        if (s < 4) heel_force += val;
        else if (s < 8) mid_force += val;
        else fore_force += val;

        weighted_x += val * sx;
        weighted_y += val * sy;

        float norm_p = std::clamp(static_cast<float>(val) / static_cast<float>(max_pressure_), 0.0f, 1.0f);
        float bar_height = std::max(0.004f, norm_p * 0.060f);

        visualization_msgs::msg::Marker sensor_bar;
        sensor_bar.header.frame_id = frame_id_;
        sensor_bar.header.stamp = stamp;
        sensor_bar.ns = "insole_sensors";
        sensor_bar.id = marker_id++;
        sensor_bar.type = visualization_msgs::msg::Marker::CYLINDER;
        sensor_bar.action = visualization_msgs::msg::Marker::ADD;

        sensor_bar.pose.position.x = sx;
        sensor_bar.pose.position.y = sy;
        sensor_bar.pose.position.z = bar_height / 2.0;
        sensor_bar.pose.orientation.w = 1.0;

        sensor_bar.scale.x = 0.016;
        sensor_bar.scale.y = 0.016;
        sensor_bar.scale.z = bar_height;

        sensor_bar.color = get_jet_color(norm_p, 0.92f);
        markers.markers.push_back(sensor_bar);
      }

      // 4. Centro de Pressão (CoP)
      visualization_msgs::msg::Marker cop;
      cop.header.frame_id = frame_id_;
      cop.header.stamp = stamp;
      cop.ns = "insole_cop";
      cop.id = marker_id++;
      cop.type = visualization_msgs::msg::Marker::SPHERE;
      cop.action = visualization_msgs::msg::Marker::ADD;
      cop.scale.x = 0.018; cop.scale.y = 0.018; cop.scale.z = 0.018;
      cop.pose.orientation.w = 1.0;

      if (total_force > 30) {
        cop.pose.position.x = weighted_x / total_force;
        cop.pose.position.y = weighted_y / total_force;
        cop.pose.position.z = 0.025;
        cop.color.r = 1.0f; cop.color.g = 0.05f; cop.color.b = 0.95f; cop.color.a = 0.95f; // Magenta brilhante
      } else {
        cop.pose.position.x = 0.0;
        cop.pose.position.y = y_base;
        cop.pose.position.z = -1.0; // Oculta abaixo do grid se sem carga
        cop.color.a = 0.0f;
      }
      markers.markers.push_back(cop);

      // 5. Rótulo e Cargas por Região Anatômica
      visualization_msgs::msg::Marker label;
      label.header.frame_id = frame_id_;
      label.header.stamp = stamp;
      label.ns = "insole_labels";
      label.id = marker_id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;

      label.pose.position.x = 0.0;
      label.pose.position.y = y_base;
      label.pose.position.z = 0.12;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.028;

      std::ostringstream ss;
      ss << foot_name << "\n";
      ss << "Total: " << total_force << " N\n";
      ss << "Heel: " << heel_force << " | Mid: " << mid_force << " | Fore: " << fore_force;
      label.text = ss.str();
      label.color.r = 1.0f; label.color.g = 1.0f; label.color.b = 1.0f; label.color.a = 0.95f;

      markers.markers.push_back(label);
    }
  }

  void build_imu_markers(
    visualization_msgs::msg::MarkerArray & markers,
    int & marker_id,
    const rclcpp::Time & stamp,
    const interfaces::msg::IMUState::SharedPtr & imu_msg)
  {
    size_t count = (imu_msg && !imu_msg->imus.empty()) ? imu_msg->imus.size() : 1;

    for (size_t i = 0; i < count; ++i) {
      double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
      std::string imu_name = "imu_" + std::to_string(i + 1);

      if (imu_msg && i < imu_msg->imus.size()) {
        const auto & imu = imu_msg->imus[i];
        imu_name = imu.name.empty() ? imu_name : imu.name;
        qw = imu.q_w;
        qx = imu.q_x;
        qy = imu.q_y;
        qz = imu.q_z;

        // Normalização de salvaguarda
        double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (norm > 1e-4) {
          qw /= norm; qx /= norm; qy /= norm; qz /= norm;
        } else {
          qw = 1.0; qx = 0.0; qy = 0.0; qz = 0.0;
        }
      }

      // Posição espacial de exibição das IMUs
      double imu_x = -0.15;
      double imu_y = (count == 1) ? 0.0 : ((i == 0) ? 0.08 : -0.08);
      double imu_z = 0.30 + static_cast<double>(i) * 0.12;

      // 1. Corpo 3D da IMU (Cubo representando a placa BNO055)
      visualization_msgs::msg::Marker cube;
      cube.header.frame_id = frame_id_;
      cube.header.stamp = stamp;
      cube.ns = "imu_body";
      cube.id = marker_id++;
      cube.type = visualization_msgs::msg::Marker::CUBE;
      cube.action = visualization_msgs::msg::Marker::ADD;

      cube.pose.position.x = imu_x;
      cube.pose.position.y = imu_y;
      cube.pose.position.z = imu_z;
      cube.pose.orientation.w = qw;
      cube.pose.orientation.x = qx;
      cube.pose.orientation.y = qy;
      cube.pose.orientation.z = qz;

      cube.scale.x = 0.045; // 45mm
      cube.scale.y = 0.035; // 35mm
      cube.scale.z = 0.008; // 8mm

      cube.color.r = 0.10f; cube.color.g = 0.18f; cube.color.b = 0.38f; cube.color.a = 0.95f; // PCB Azul/Preta
      markers.markers.push_back(cube);

      // 2. Tríade de Eixos Orientados (Setas X, Y, Z rodando pelo quaternion)
      const double AXIS_LEN = 0.08;
      const double SHAFT_D = 0.005;
      const double HEAD_D = 0.012;

      struct AxisDef {
        double vx, vy, vz;
        float r, g, b;
      };
      AxisDef axes[3] = {
        {AXIS_LEN, 0.0, 0.0, 1.0f, 0.1f, 0.1f}, // X = Vermelho
        {0.0, AXIS_LEN, 0.0, 0.1f, 1.0f, 0.1f}, // Y = Verde
        {0.0, 0.0, AXIS_LEN, 0.1f, 0.2f, 1.0f}  // Z = Azul
      };

      for (int a = 0; a < 3; ++a) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = frame_id_;
        arrow.header.stamp = stamp;
        arrow.ns = "imu_axes";
        arrow.id = marker_id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose.orientation.w = 1.0;

        geometry_msgs::msg::Point p0;
        p0.x = imu_x; p0.y = imu_y; p0.z = imu_z;

        geometry_msgs::msg::Point p_rot = rotate_vector_by_quaternion(
          axes[a].vx, axes[a].vy, axes[a].vz, qw, qx, qy, qz);

        geometry_msgs::msg::Point p1;
        p1.x = imu_x + p_rot.x;
        p1.y = imu_y + p_rot.y;
        p1.z = imu_z + p_rot.z;

        arrow.points.push_back(p0);
        arrow.points.push_back(p1);

        arrow.scale.x = SHAFT_D;
        arrow.scale.y = HEAD_D;
        arrow.scale.z = 0.02;

        arrow.color.r = axes[a].r;
        arrow.color.g = axes[a].g;
        arrow.color.b = axes[a].b;
        arrow.color.a = 0.95f;
        markers.markers.push_back(arrow);
      }

      // 3. Ângulos de Euler e Rótulo Textual
      double sinr_cosp = 2.0 * (qw * qx + qy * qz);
      double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
      double roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

      double sinp = 2.0 * (qw * qy - qz * qx);
      double pitch = (std::abs(sinp) >= 1.0) ? std::copysign(90.0, sinp) : std::asin(sinp) * 180.0 / M_PI;

      double siny_cosp = 2.0 * (qw * qz + qx * qy);
      double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
      double yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;

      visualization_msgs::msg::Marker imu_label;
      imu_label.header.frame_id = frame_id_;
      imu_label.header.stamp = stamp;
      imu_label.ns = "imu_labels";
      imu_label.id = marker_id++;
      imu_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      imu_label.action = visualization_msgs::msg::Marker::ADD;

      imu_label.pose.position.x = imu_x;
      imu_label.pose.position.y = imu_y;
      imu_label.pose.position.z = imu_z + 0.09;
      imu_label.pose.orientation.w = 1.0;
      imu_label.scale.z = 0.025;

      std::ostringstream ss;
      ss << "🧭 " << imu_name << "\n";
      ss << "R: " << std::fixed << std::setprecision(1) << roll << "° | ";
      ss << "P: " << std::setprecision(1) << pitch << "° | ";
      ss << "Y: " << std::setprecision(1) << yaw << "°";
      imu_label.text = ss.str();
      imu_label.color.r = 0.3f; imu_label.color.g = 0.85f; imu_label.color.b = 1.0f; imu_label.color.a = 0.95f;

      markers.markers.push_back(imu_label);
    }
  }

  std::string frame_id_;
  double max_pressure_;
  std::chrono::steady_clock::time_point start_time_;

  std::mutex data_mutex_;
  RateEstimator rate_pressure_;
  RateEstimator rate_imu_;
  interfaces::msg::PressureState::SharedPtr last_pressure_msg_;
  interfaces::msg::IMUState::SharedPtr last_imu_msg_;

  std::vector<std::pair<float, float>> left_foot_contour_;
  std::vector<std::pair<float, float>> left_sensor_coords_;

  rclcpp::Subscription<interfaces::msg::PressureState>::SharedPtr sub_pressure_;
  rclcpp::Subscription<interfaces::msg::IMUState>::SharedPtr sub_imu_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_pressure_hz_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_imu_hz_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorVisualizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
