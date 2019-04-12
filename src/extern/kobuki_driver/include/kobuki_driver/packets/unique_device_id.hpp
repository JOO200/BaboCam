/**
 * @file /include/kobuki_driver/packets/unique_device_id.hpp
 * @author Younghun Ju <yhju@yujinrobot.com> <yhju83@gmail.com>
 * @brief Module for handling of unique device id request packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki_core/hydro-devel/kobuki_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_UDID_DATA_HPP__
#define KOBUKI_UDID_DATA_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include <libserial/SerialStream.h>
#include "../packet_handler/payload_base.hpp"
#include "../packet_handler/payload_headers.hpp"
#include "syslog.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kobuki
{

/*****************************************************************************
** Interface
*****************************************************************************/

class UniqueDeviceID : public packet_handler::payloadBase
{
public:
  UniqueDeviceID() : packet_handler::payloadBase(false, 12),data() {};

  struct Data {
    uint32_t udid0;
    uint32_t udid1;
    uint32_t udid2;
  } data;

  // methods
  bool serialise(SerialDataBuffer & byteStream) override
  {
    buildBytes(Header::UniqueDeviceID, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.udid0, byteStream);
    buildBytes(data.udid1, byteStream);
    buildBytes(data.udid2, byteStream);
    return true;
  }

  bool deserialise(SerialDataBuffer & byteStream) override {
    if (byteStream.size() < length+2)
    {
      //std::cout << "kobuki_node: kobuki_udid: deserialise failed. not enough byte stream." << std::endl;
      return false;
    }

    unsigned char header_id, length_packed;
    buildVariable(header_id, byteStream);
    buildVariable(length_packed, byteStream);
    if( header_id != Header::UniqueDeviceID ) {
        syslog(LOG_ERR, "Wrong header found: needed[%d], foud[%d] ", Header::UniqueDeviceID, header_id);
        return false;
    }
    if( length_packed != length ) {
        syslog(LOG_ERR, "Wrong length found: needed[%d], foud[%d] ", length, length_packed);
        return false;
    }

    buildVariable(data.udid0, byteStream);
    buildVariable(data.udid1, byteStream);
    buildVariable(data.udid2, byteStream);

    //showMe();
    return constrain();
  }

  bool constrain()
  {
    return true;
  }

  void showMe()
  {
  }
};

} // namespace kobuki

#endif /* KOBUKI_UDID_DATA_HPP__ */

