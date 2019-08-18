//
// Created by johannes on 5/2/19.
//

#ifndef BABOCAM_MOVABLEDEVICE_HPP
#define BABOCAM_MOVABLEDEVICE_HPP

#include <stdint.h>
#include "../extern/kobuki_driver/include/kobuki_driver/kobuki.hpp"

/**
 * Abstrakte Klasse für ein bewegbares Gefährt.
 * Wird diese Klasse genutzt, werden alle entsprechenden Werte in das Log geschrieben.
 */
class MovableDevice {
public:
    MovableDevice() = default;
    virtual void setMove(int16_t speed, int16_t ratio) {
        syslog(LOG_INFO, "MoveCommand received. Speed[%d], Ratio[%d]", speed, ratio);
    }
};

/**
 * Bewegbares Gefährt des Typs "Kobuki Turtlebot2"
 */
class KobukiDevice : public MovableDevice {
public:
    /**
     * ctor
     * @param device Referenz auf den Turtlebot-Treiber
     */
    explicit KobukiDevice(kobuki::Kobuki & device):m_device(device) {}
    void setMove(int16_t speed, int16_t ratio) override {
        m_device.setBaseControl(speed, ratio);
    }
private:
    kobuki::Kobuki & m_device;
};


#endif //BABOCAM_MOVABLEDEVICE_HPP
