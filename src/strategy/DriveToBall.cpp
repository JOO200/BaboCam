//
// Created by johannes on 5/2/19.
//

#include <syslog.h>
#include "DriveToBall.hpp"
#include "../math/own_math.hpp"

#define MIN_ANGLE 0.2

void DriveToBall::step(Context *context, MovableDevice * kobuki, std::atomic<bool>& m_stop) {
    context->getDataMutex().lock();
    double speed(NAN), ratio(NAN);

    if(context->getBall().getAngle() == 0 && context->getBall().getDistance() == 0) {
        syslog(LOG_ERR, "Invalid context found.");
        kobuki->setMove(0, 0);
        context->getDataMutex().unlock();
        return;
    }
    static uint8_t start_counter = 0;
    start_counter++;
    if(start_counter < 5) {
        kobuki->setMove(0, 0);
        context->getDataMutex().unlock();
        return;
    }
    if(std::abs(context->getBall().getAngle()) < MIN_ANGLE) {
        double distance = context->getBall().getDistance();
        speed = range(distance*100, 100, 200);
        ratio = 0;
    } else {
        ratio = 1;
        speed = context->getBall().getAngle() > 0 ? -70 : 70;
    }
    if(ratio == 1) {
        speed = absRange(speed, 25.0, 70.0);
    }

    if(context->getBall().getDistance() < 0.25) {
        context->setState(Context::State::KICK);
    }
    context->getDataMutex().unlock();
    kobuki->setMove(speed, ratio);
}
