#ifndef PRESSURE_DRIVER_HPP
#define PRESSURE_DRIVER_HPP

#include "common_serial/serial_handler.hpp"
#include "driver/pressure_driver.hpp"

#define MAX_BUFFER_COLLECT          6 * 16
#define BUFFER_SIZE                 8 * 16 * 5

class PressureDriver : public IPressureDriver
{
    public:
        explicit PressureDriver();

        int init(std::string device, int baudrate) override;
        int get_data(std::vector<uint16_t>& data) override;

    private:
        std::vector<uint16_t> parse_numbers_from_string(const std::string& input);
        bool read_c_string(char* buffer, size_t max_size);
        bool sync_start_line();

        std::unique_ptr<SerialHandler> transport_;
};

#endif // PRESSURE_DRIVER_HPP
