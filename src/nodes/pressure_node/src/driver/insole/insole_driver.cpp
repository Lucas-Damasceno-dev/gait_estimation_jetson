#include "driver/insole/insole_driver.hpp"
#include <sstream>
#include <string>

InsoleDriver::InsoleDriver() {}

int InsoleDriver::init(std::string device, int baudrate)
{
    transport_ = std::make_unique<SerialHandler>();

    // cria o fd
    if (transport_->init(device.c_str()) < 0)
        return -1; 

    // configurações do fd
    transport_->setDefaultConfig();
    transport_->setBaudRate(baudrate);

    return 0;
}

std::vector<uint16_t> InsoleDriver::parse_numbers_from_string(
    const std::string& input)
{
    std::vector<uint16_t> values;
    std::string cleaned = input;
    for (char& c : cleaned) {
        if (c == ',' || c == ';' || c == '\t' || c == '\r') {
            c = ' ';
        }
    }
    std::stringstream ss(cleaned);
    std::string token;

    while (ss >> token) {
        try {
            values.push_back(static_cast<uint16_t>(std::stoul(token)));
        } catch (...) {
            continue;
        }
    } 
    return values;
}

int InsoleDriver::get_data(std::vector<uint16_t>& data)
{
    char tmp[256];
    ssize_t n;

    // busca bloco de dados no buffer do canal específico
    while ((n = transport_->readData(tmp, sizeof(tmp))) > 0)
        rx_buffer_.append(tmp, n);

    // encontra a última mensagem completa que estava no buffer
    std::string last_line;
    size_t pos;
    while ((pos = rx_buffer_.find('\n')) != std::string::npos)
    {
        last_line = rx_buffer_.substr(0, pos);
        rx_buffer_.erase(0, pos + 1); // limpa o que já leu
    }

    if (rx_buffer_.size() > 2048) {
        rx_buffer_.clear();
    }

    if (last_line.empty())
        return -1; 

    // converte a linha em um vetor de números
    data = parse_numbers_from_string(last_line);

    if (data.empty())
        return -2; 

    return 0; 
}
