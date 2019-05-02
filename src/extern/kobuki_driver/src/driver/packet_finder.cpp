/**
 * @file /kobuki_driver/src/driver/packet_finder.cpp
 *
 * @brief Packet handling implementation.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki_core/hydro-devel/kobuki_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <sstream>
#include <iostream>
#include <syslog.h>
#include <libserial/SerialPort.h>
#include "../../include/kobuki_driver/packet_handler/packet_finder.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/

    PacketFinderBase::PacketFinderBase() :
            verbose(false) {
    }


/*****************************************************************************
** Public
*****************************************************************************/

    void PacketFinderBase::configure(const std::string &sigslots_namespace,
                                     const BufferType &putStx, const BufferType &putEtx, unsigned int sizeLengthField,
                                     unsigned int sizeMaxPayload, unsigned int sizeChecksumField,
                                     bool variableSizePayload) {
        size_length_field = sizeLengthField;
        variable_size_payload = variableSizePayload;
        size_max_payload = sizeMaxPayload;
        size_payload = variable_size_payload ? 0 : sizeMaxPayload;
        size_checksum_field = sizeChecksumField;
        buffer = BufferType(size_stx + size_length_field + size_max_payload + size_checksum_field + size_etx);

//  sig_warn.connect(sigslots_namespace + std::string("/ros_warn"));
//  sig_error.connect(sigslots_namespace + std::string("/ros_error"));

        //todo; exception
        // Problem1: size_length_field = 1, vairable_size_payload = false

        buffer.clear();
    }

    void PacketFinderBase::enableVerbose() {
        verbose = true;
    }

    void PacketFinderBase::getBuffer(BufferType &bufferRef) {
        bufferRef = buffer;
    }

    void PacketFinderBase::getPayload(BufferType &bufferRef) {
        bufferRef.clear();
        bufferRef.resize(buffer.size() - size_stx - size_etx - size_length_field - size_checksum_field);
        for (unsigned int i = size_stx + size_length_field; i < buffer.size() - size_etx - size_checksum_field; i++) {
            bufferRef.push_back(buffer[i]);
        }
    }

/**
 * Checks for incoming packets.
 *
 * @param incoming
 * @param numberOfIncoming
 * @return bool : true if a valid incoming packet has been found.
 */
    SerialDataBuffer & PacketFinderBase::update(LibSerial::SerialPort &dataStream) {
        // clearBuffer = 0, waitingForStx, waitingForPayloadSize, waitingForPayloadToEtx, waitingForEtx,
        // std::cout << "update [" << numberOfIncoming << "][" << state << "]" << std::endl;

        syslog(LOG_INFO, "Wait for stx start.");
        WaitForStx(dataStream);
        syslog(LOG_INFO, "Wait for stx end.");
        waitForPayloadSize(dataStream);
        syslog(LOG_INFO, "Payload Size %d.", size_payload);
        buffer.clear();
        syslog(LOG_INFO, "Read Buffer");
        dataStream.Read(buffer, size_payload, 0);
        syslog(LOG_INFO, "Read Checksum");
        unsigned char check_sum;
        dataStream.ReadByte(check_sum, 0);

        //syslog(LOG_INFO, "Buffer 0: 0x%02x", buffer[0]);
        std::string concat;
        std::stringstream ss;
        for(auto & num : buffer) {
            ss << std::hex << (int)num << " ";
        }
        concat = ss.str();
        //syslog(LOG_INFO, "Read: %s", concat.c_str());

        return buffer;
    }

/*****************************************************************************
** Protected
*****************************************************************************/

    bool PacketFinderBase::WaitForStx(LibSerial::SerialPort & incoming) {
        bool found_stx(true);

        // add incoming datum
        //buffer.push_back(datum);
        while(true) {
            unsigned char read;
            incoming.ReadByte(read, 0);
            if(read == 0xAA) {
                incoming.ReadByte(read, 0);
                if(read == 0xAA) {
                    incoming.ReadByte(read, 0);
                }
                if(read == 0x55) {
                    break;
                }
            } else if(read == 0x55) {
                break;
            }
        }
        return true;
    }

    bool PacketFinderBase::waitForPayloadSize(LibSerial::SerialPort & incoming) {
        // push data

        unsigned char first_byte;
        incoming.ReadByte(first_byte, 0);

        size_payload = static_cast<unsigned int>(first_byte);
        if (verbose) {
            syslog(LOG_DEBUG, "[payloadSize: %d]", size_payload);
        }

        return true;
    }
} // namespace kobuki
