#ifndef SERIAL_HANDLER_HPP 
#define SERIAL_HANDLER_HPP

#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
#include <string>

#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "itransport.hpp"


// Definições de termios2 baseadas no Dynamixel SDK (ROBOTIS)
struct termios2 {
  tcflag_t c_iflag;       /* input mode flags */
  tcflag_t c_oflag;       /* output mode flags */
  tcflag_t c_cflag;       /* control mode flags */
  tcflag_t c_lflag;       /* local mode flags */
  cc_t c_line;            /* line discipline */
  cc_t c_cc[19];          /* control characters */
  speed_t c_ispeed;       /* input speed */
  speed_t c_ospeed;       /* output speed */
};

/* 
 * Fonte:
 * https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
*/

class SerialHandler : public ITransport
{
    public:
        enum class Parity { NONE, EVEN, ODD };
        enum class StopBits { ONE, TWO };

        SerialHandler();
        virtual ~SerialHandler();

        int init(const char* device);
        int setBaudRate(int speed);
        int setDefaultConfig();
        
        ssize_t readData(uint8_t* buffer, size_t size) override
        {
            return readGenericData(buffer, size);
        }

        template<typename T>
        ssize_t readData(T *buffer, size_t size)
        {
            return readGenericData(buffer, size);
        }
        
        ssize_t writeData(const uint8_t* buffer, size_t size) override;
        void clearBuffer();
        
    private:
        
        ssize_t readGenericData(void *buffer, size_t size);
        
        int fd_;
};

#endif // SERIAL_HANDLER_HPP
