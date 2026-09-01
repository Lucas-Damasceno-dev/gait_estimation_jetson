#include "common_serial/serial_handler.hpp"

#include <sys/ioctl.h>
#include <chrono>
#include <thread>

#ifndef TCGETS2
#define TCGETS2     _IOR('T', 0x2A, struct termios2)
#endif
#ifndef TCSETS2
#define TCSETS2     _IOW('T', 0x2B, struct termios2)
#endif
#ifndef BOTHER
#define BOTHER      0010000
#endif

SerialHandler::SerialHandler() : fd_(-1) {}

int SerialHandler::init(const char* device)
{
    fd_ = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cout << "Error " << errno 
            << " from open: " << std::strerror(errno) 
            << std::endl;
        return -1;
    }
    
    return 0;
}

int SerialHandler::setDefaultConfig()
{
    struct termios config;
    
    if (fd_ < 0) return -EBADF;

    // configurações
    if (tcgetattr(fd_, &config) != 0) {
        fprintf(stderr, "Erro em tcgetattr(): %s" , strerror(errno));
        fflush(stderr);

        return -1;
    }

    config.c_cflag = CS8 | CLOCAL | CREAD;
    config.c_iflag = IGNPAR;
    config.c_oflag      = 0;
    config.c_lflag      = 0;
    config.c_cc[VTIME]  = 0;
    config.c_cc[VMIN]   = 0;

    // aplica as configs
    if (tcsetattr(fd_, TCSANOW, &config) != 0) {
        fprintf(stderr, "Erro ao aplicar configuração: %s", strerror(errno));
        fflush(stderr);
        return -1;
    }
    
    return 0;
}

int SerialHandler::setBaudRate(int baud)
{
    struct termios2 options;

    if (fd_ < 0) return -EBADF;
    
    if (ioctl(fd_, TCGETS2, &options) != -1)
    {
        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_ispeed = baud;
        options.c_ospeed = baud;

        if (ioctl(fd_, TCSETS2, &options) != -1)
            return 0;
        return -errno; 
    }
    return -errno; 
}

ssize_t SerialHandler::writeData(const uint8_t* buffer, size_t size)
{
    if (fd_ < 0) return -1;

    ssize_t n = write(fd_, buffer, size);
    if (n < 0) return -1;

    return n;
}

ssize_t SerialHandler::readGenericData(void *buffer, size_t size)
{
    if (fd_ < 0) return -1;
    ssize_t n = read(fd_, buffer, size);
    if (n < 0) return -1;
    return n;
}

void SerialHandler::clearBuffer()
{
    tcflush(fd_, TCIFLUSH);
}

SerialHandler::~SerialHandler()
{
    if (fd_ >= 0) close(fd_);
}
