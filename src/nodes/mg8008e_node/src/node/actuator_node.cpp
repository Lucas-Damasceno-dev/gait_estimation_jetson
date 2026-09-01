#include "node/actuator_node.hpp"

#include "control/node/parameter_manager.hpp"

#include "driver/actuator_factory.hpp"

#include <vector>

using namespace std::chrono_literals;

ActuatorNode::ActuatorNode(const rclcpp::NodeOptions& options) 
    : Node("actuator_node", options)
{   
    init_driver();

    // TODO: verificar se os ids fornecidos pelo yaml
    // são de atuadores realmente conectados
    setup_node();
}

void ActuatorNode::init_driver() {
    parameter_manager_ = std::make_shared<ParameterManager>(this);
    actuator_driver_ = ActuatorFactory::createMG8008E();

    // inicializa porta serial inserida nos parâmetros do yaml
    auto init_response = actuator_driver_->init(
        parameter_manager_->get_usb_port(), 
        parameter_manager_->get_baudrate());
    
    if (init_response != 0)
    {
        RCLCPP_FATAL(this->get_logger(), 
            "Falha na inicialização do hardware serial na porta %s.", 
            parameter_manager_->get_usb_port().c_str());
        throw std::runtime_error("Falha ao inicializar ActuatorNode");
    }
    RCLCPP_INFO(this->get_logger(), "Inicializaço do hardware na porta %s realizada com sucesso",
        parameter_manager_->get_usb_port().c_str());

    // inicializando cada atuador
    auto ids = parameter_manager_->get_ids();
    size_t init_counter = ids.size();
    for (size_t idx = 0; idx < ids.size(); idx++) {
		if (actuator_driver_->setup_driver(ids[idx]) != 0)
        {
			RCLCPP_ERROR(this->get_logger(), "Falha na configuração do atuador %s.", 
                parameter_manager_->get_names()[idx].c_str());            
            init_counter--;
        }
	}
    
    // se nenhum atuador foi inicializado não executa o nó
    if (init_counter == 0)
    {
        RCLCPP_FATAL(this->get_logger(), "Falha na inicialização dos atuadores.");
        throw std::runtime_error("Falha ao inicializar ActuatorNode");
    }

}

void ActuatorNode::setup_node() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .durability_volatile();

    // recebe dados de comando contínuo. Ex: posição alvo
    actuator_subscriber_ = this->create_subscription<MG8008ECommand>(
         "actuator/command", qos, 
        [this](const MG8008ECommand::SharedPtr msg) {
            this->set_angle(this->read_angle_msg(msg));
        });

    state_publisher_ = this->create_publisher<MG8008EState>(
         "actuator/state", qos);
    time_publisher_ = this->create_publisher<Time>(
         "actuator/time", qos);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameter_manager_->get_update_rate()), 
        [this]() {
            publish_actuator_state();
        });
    
    RCLCPP_INFO(this->get_logger(), "Sucesso ao inicializar ActuatorNode.");
}

MG8008EState ActuatorNode::read_actuator_data(Time& time_data)
{
    MG8008EState state_data;
    
    const auto& ids = parameter_manager_->get_ids();
    const auto& names = parameter_manager_->get_names();

    {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        
        for (size_t idx = 0; idx < ids.size(); idx++)
        {
            double angle;
            int result;

            auto duration = measure_micros([&]() {
                result = actuator_driver_->get_angle(ids[idx], angle);
            });
            
            time_data.names.push_back(names[idx]);
            time_data.times.push_back(duration);
            
            if (result == 0) {
                state_data.names.push_back(names[idx]);
                state_data.angles.push_back(static_cast<int32_t>(angle));

            } else {
                RCLCPP_ERROR(this->get_logger(), "Falha na leitura do atuador %s. Erro: %d", 
                    names[idx].c_str(), static_cast<int>(result));
            }
        }
    }

    return state_data;
}

void ActuatorNode::set_angle(const std::vector<ActuatorData>& actuator_data)
{
    std::lock_guard<std::mutex> lock(driver_mutex_);
    
    // define o angulo para cada atuador
    for (size_t idx = 0; idx < actuator_data.size(); idx++)
    {
        if (actuator_data[idx].speed <= 0) {
            RCLCPP_ERROR(this->get_logger(), 
                "Falha no envio de Goal Position para o atuador %s. A velocidade deve ser um valor positivo", 
                actuator_data[idx].name.c_str());
            continue;
        }

        RCLCPP_INFO(this->get_logger(), "\nID: %d\nAngle: %d\nSpeed: %d", 
        actuator_data[idx].id, actuator_data[idx].angle, actuator_data[idx].speed);
        auto result = actuator_driver_->set_angle(
            actuator_data[idx].id, 
            actuator_data[idx].angle,
            actuator_data[idx].speed);

        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Falha no envio de Goal Position para o atuador %s. Erro: %d", 
            actuator_data[idx].name.c_str(), static_cast<int>(result));
        }
    }
}

std::vector<ActuatorData> ActuatorNode::read_angle_msg(const MG8008ECommand::SharedPtr msg) {
    if (msg->names.empty() || msg->angles.empty()) return {};

    std::vector<ActuatorData> actuator_data_list;

    size_t n = std::min(msg->names.size(), msg->angles.size()); 
    for (size_t idx = 0; idx < n; idx++)
    {
        auto id = parameter_manager_->get_id_by_name(msg->names[idx]);
        if (id == -1) continue;

        ActuatorData actuator_data;
        actuator_data.id = static_cast<uint8_t>(id);
        actuator_data.name = msg->names[idx];
        actuator_data.angle = msg->angles[idx];
        actuator_data.speed = msg->speeds[idx];

        actuator_data_list.push_back(actuator_data);
    }

    return actuator_data_list;
}

void ActuatorNode::publish_actuator_state()
{
    Time time_msg;
    MG8008EState state_msg;

    auto duration = measure_micros([&]() {
        state_msg = read_actuator_data(time_msg);
    });
    
    if (state_msg.names.empty()) return;

    state_msg.header.stamp = this->get_clock()->now();
    state_publisher_->publish(state_msg);

    time_msg.total_time = duration;
    time_publisher_->publish(time_msg);
}

ActuatorNode::~ActuatorNode() {
    RCLCPP_INFO(this->get_logger(), "Encerrando ActuatorNode e limpando recursos...");

    // para o timer
    if (timer_) {
        timer_->cancel();
    }

    // TODO: desconectar todos os motores
    auto ids = parameter_manager_->get_ids();
    for (size_t idx = 0; idx < ids.size(); idx++)
        actuator_driver_->disconnect(ids[idx]);
};
