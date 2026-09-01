#ifndef ITRANSPORT_HPP
#define ITRANSPORT_HPP

#include <cstdint>   // uint8_t
#include <cstddef>   // size_t
#include <sys/types.h> // ssize_t

class ITransport
{
public:
    virtual ~ITransport() = default;

    virtual ssize_t writeData(const uint8_t* data, size_t size) = 0;
    virtual ssize_t readData(uint8_t* data, size_t size) = 0;
};

#endif // ITRANSPORT_HPP