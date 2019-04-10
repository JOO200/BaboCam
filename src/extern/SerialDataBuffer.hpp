//
// Created by johannes on 4/10/19.
//

#ifndef BABOCAM_SERIALDATABUFFER_HPP
#define BABOCAM_SERIALDATABUFFER_HPP

#include <vector>
#include <stdint.h>

class SerialDataBuffer : public std::vector<uint8_t> {
public:
    explicit SerialDataBuffer(long length) : std::vector<uint8_t>(length) { }

    explicit SerialDataBuffer() : std::vector<uint8_t>() { }

    char snextc() {
        char next = front();
        erase(begin());
        return next;
    }

    void sputc(char put) {
        push_back(put);
    }
};

#endif //BABOCAM_SERIALDATABUFFER_HPP
