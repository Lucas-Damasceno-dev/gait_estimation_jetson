#include "driver/dynamixel/dynamixel_driver.hpp"
#include "common_serial/serial_handler.hpp"

#include <chrono>
#include <cstring>

DynamixelDriver::DynamixelDriver() {}

// Inicia comunicação com a porta serial
int DynamixelDriver::init(std::string device, int baudrate)
{
    transport_ = std::make_unique<SerialHandler>();
    if (transport_->init(device.c_str()) < 0)
        return -1; 
    transport_->setDefaultConfig();
    transport_->setBaudRate(baudrate);

    return 0;
}

int DynamixelDriver::set_torque(uint8_t id, uint8_t enable_torque)
{
    std::vector<uint8_t> params = {TORQUE_ADDR, enable_torque};

    std::vector<uint8_t> packet = get_packet(id, WRITE_INSTR, params);

    if (transport_->writeData(packet.data(), packet.size()) < 0) 
	return -1;

    // le o status pra evitar erros
    StatusPacket status;
    if (read_status(id, status) < 0)
        return -1;
	
    return 0;
}

int DynamixelDriver::set_goal_position(uint8_t id, uint16_t goal_position)
{
    uint8_t goal_pos_lsb = goal_position & 0xFF;
    uint8_t goal_pos_msb = (goal_position >> 8) & 0xFF;
    std::vector<uint8_t> params = {GOAL_POS_ADDR, goal_pos_lsb, goal_pos_msb};

    std::vector<uint8_t> packet = get_packet(id, WRITE_INSTR, params);

    if (transport_->writeData(packet.data(), packet.size()) < 0) 
	return -1;

    // le o status pra evitar erros
    StatusPacket status;
    if (read_status(id, status) < 0)
        return -1;
    
    return 0;
}

int DynamixelDriver::get_current_position(uint8_t id, uint16_t& current_position)
{
    std::vector<uint8_t> params = {CURRENT_POS_ADDR, 2};


    std::vector<uint8_t> packet = get_packet(id, READ_INSTR, params);

    if (transport_->writeData(packet.data(), packet.size()) < 0) 
		return -1;

    StatusPacket status;
    if (read_status(id, status) < 0)
        return -1;

    current_position = static_cast<uint16_t>(status.params[0]) |
                   (static_cast<uint16_t>(status.params[1]) << 8);

    return 0;
}

std::vector<uint8_t> DynamixelDriver::get_packet(
    uint8_t id, uint8_t instr, const std::vector<uint8_t>& params)
{
    std::vector<uint8_t> packet(PARAMETER_POS+params.size()+1);

    packet[PREAMBLE_POS]            = 0xFF;
	packet[PREAMBLE_POS+1]          = 0xFF;
    packet[ID_POS]                  = id;
    packet[INSTRUCTION_POS]         = instr;

    // adição de parâmetros no pacote
	for (int i = 0; i<params.size(); i++)
		packet[PARAMETER_POS+i]     = params[i];
    packet[LENGTH_POS]              = params.size() + 2;

    // calculo do checksum
    uint16_t sum = 0;
	for (uint8_t i = ID_POS; i < packet.size()-1; i++)
		sum += packet[i];
	packet[packet.size()-1]         = ~(sum & 0xFF);

    return packet;
}

// Monta pacote de dados que é enviado pelo dynamixel
int DynamixelDriver::read_packet(std::array<uint8_t, RXPACKET_MAX_LEN>& packet)
{
	size_t read_size = 0;
	size_t wait_length = 6; // tamanho minimo (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)

	auto start = std::chrono::steady_clock::now();
	constexpr auto TIMEOUT = std::chrono::milliseconds(20);

	while (true)
	{
		if (std::chrono::steady_clock::now() - start > TIMEOUT)
			return -2;

		ssize_t n = transport_->readData(packet.data() + read_size,
			wait_length - read_size);

		if (n > 0)
			read_size += static_cast<size_t>(n);

		// se ainda não foi lido 6 bytes, volta a ler
		if (read_size < wait_length)
			continue;

		// encontra a posição do header no pacote
		uint8_t idx = 0;
		for (idx = 0; idx < (read_size - 1); idx++)
		{
			if (packet[idx] == 0xFF && packet[idx + 1] == 0xFF)
				break;
		}
		
		// se o header estiver no inicio
		if (idx == 0)
		{
			if (packet[2] > 0xFD ||
				packet[3] > RXPACKET_MAX_LEN ||
				packet[4] > 0x7F)
			{
				std::memmove(packet.data(), packet.data() + 1, read_size - 1);
				read_size -= 1;
				wait_length = 6; // reseta para busca de header
				continue;
			}

			// recalcula o tamanho real do pacote
			// para comportar os parâmetros
			size_t total_expected = packet[3] + 4;

			if (wait_length != total_expected)
			{
				wait_length = total_expected;
				// volta para o loop pra ler o resto dos parâmetros
				continue; 
			}

			// verifica o checksum
			uint8_t checksum = 0;
			for (size_t i = 2; i < wait_length - 1; i++)
				checksum += packet[i];
			checksum = ~checksum;

			// verifica se o checksum bate
			if (packet[wait_length - 1] == checksum)
				return 0;
			else	
				return -1;
		} else 
		{
			// move os dados, fazendo o header ficar no inicio
			std::memmove(packet.data(), packet.data() + idx, read_size - idx);
			read_size -= idx;
		}
	}
}

// Monta um status packet com valores úteis
// advindos do pacote montado por read_packet()
int DynamixelDriver::read_status(uint8_t id, StatusPacket& out)
{
	std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;

	if (read_packet(rxbuffer) != 0) return -1;
	if (id != rxbuffer[ID_POS]) return -1;

	uint8_t length = rxbuffer[LENGTH_POS];
	uint8_t error = rxbuffer[ERROR_POS];

	out.id = rxbuffer[ID_POS];
	out.error = error;

	if (error != 0) return -1; 

    // adiciona os parâmetros
	for (uint8_t i = 0; i < length-2; i++)
	{
		out.params[i] = rxbuffer[PARAMETER_POS+i];
	}

	return 0;
}
