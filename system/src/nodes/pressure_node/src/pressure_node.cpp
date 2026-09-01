#include <vector>
#include <sstream>

#include "pressure_node.hpp"
#include "driver/pressure_factory.hpp"

using namespace std::chrono_literals;

PressureNode::PressureNode(const rclcpp::NodeOptions& options) 
    : Node("pressure_node", options)
{
    init_driver();
    setup_node();
}

void PressureNode::init_driver() 
{
    parameter_manager_ = std::make_shared<ParameterManager>(this);
    pressure_drivers_.resize(parameter_manager_->get_ids().size());

    // inicializa o sensor de cada porta declarada
    for (size_t idx = 0; idx < parameter_manager_->get_ids().size(); idx++)
    {
        pressure_drivers_[idx] = PressureFactory::create_pressure();

        auto init_response = pressure_drivers_[idx]->init(
            parameter_manager_->get_usb_ports()[idx], 
            parameter_manager_->get_baudrate());
        
        // se apenas uma porta falhar, o nó não executa
        if (init_response < 0) {
            RCLCPP_FATAL(this->get_logger(), 
                "Falha na inicialização do hardware serial na porta %s.", 
                parameter_manager_->get_usb_ports()[idx].c_str());
            throw std::runtime_error("Falha ao inicializar PressureNode"); 
        }
    }
}

void PressureNode::setup_node() 
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    publisher_ = this->create_publisher<PressureState>(
        "pressure/state", qos);

    time_publisher_ = this->create_publisher<Time>(
        "pressure/time", qos);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameter_manager_->get_update_rate()), 
        [this]() {
            publish_pressure_state();
        });

    RCLCPP_INFO(this->get_logger(), "Sucesso ao inicializar PressureNode.");
}

PressureState PressureNode::read_pressure_data(Time& time_data)
{
    PressureState state_data;

    const auto& ids = parameter_manager_->get_ids();
    const auto& names = parameter_manager_->get_names();

    size_t min_size = std::min(ids.size(), names.size());

    for (size_t idx = 0; idx < min_size; idx++)
    {
        std::vector<uint16_t> data;
        int result;

        // pega os dados da palmilha na porta específica
        auto duration = measure_micros([&]() {
            result = pressure_drivers_[idx]->get_data(data);
        });

        time_data.names.push_back(names[idx]);
        time_data.times.push_back(duration);

        if (result != 0)
            continue; // se não conseguir os dados não cria a msg

        // cria a mensagem do sensor em específico
        PressureData pressure_data;
        pressure_data.pressures.reserve(data.size());
            
        size_t id_counter = 0;
        for (auto value: data) {
            PressureUnitSensor pressure_unit;
            pressure_unit.id = ++id_counter;
            pressure_unit.pressure = static_cast<int16_t>(value);
            
            pressure_data.pressures.push_back(pressure_unit);
        }

        // adiciona dados do sensor específico na mensagem "geral"
        state_data.pressures.push_back(pressure_data);
        state_data.names.push_back(parameter_manager_->get_names()[idx]);
    }

    return state_data;
}

void PressureNode::publish_pressure_state()
{
    Time time_msg;
    PressureState state_msg;

    // pega a mensagem criada
    auto duration = measure_micros([&]() {
        state_msg = read_pressure_data(time_msg);
    });
    
    // se nenhum sensor tiver funcionado 
    // (se não criou nenhuma mensagem)
    // não publica nada
    if (state_msg.names.empty()) return;

    state_msg.header.stamp = this->get_clock()->now();
    publisher_->publish(state_msg);

    // publica mensagem de tempo
    time_msg.total_time = duration;
    time_publisher_->publish(time_msg);
}

PressureNode::~PressureNode() = default;
