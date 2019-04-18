/*
 * PathFinder.cpp
 *
 *  Created on: Mar 4, 2019
 *      Author: johannes
 */




#include <syslog.h>
#include <math.h>
#include "PathFinder.hpp"

#define MIN_ANGLE 3*M_PI/180
#define MIDDLE_ANGLE 10*M_PI/180

void PathFinder::run() {
    std::unique_lock<std::mutex> lck(context->getM());

    while(!m_stop) {
        std::cv_status returnVal = context->getCond().wait_for(lck, std::chrono::seconds(2));
        if(returnVal == std::cv_status::timeout) {
            syslog(LOG_ERR, "Timeout");
            break;
        }
        if(m_stop) {
            syslog(LOG_INFO, "Stopping Pathfinding");
            break;
        }
        if(context->getBall().getAngle() == 0 && context->getBall().getDistance() == 0) {
            syslog(LOG_ERR, "Invalid context found.");
            break;
        }
        double speed, ratio;

        if(std::abs(context->getBall().getAngle()) < MIN_ANGLE) {
            double distance = context->getBall().getDistance();
            speed = std::max(distance, 50.0);
            ratio = 0;
        } else if(std::abs(context->getBall().getAngle()) < MIDDLE_ANGLE) {
            ratio = 1;
            speed = MIDDLE_ANGLE * 230/2;
        } else {
            float angle = context->getBall().getAngle();
            ratio = 1;
            speed = angle * 230/10;
        }
        device->setBaseControl((int16_t)speed, (int16_t)ratio);
    }
STOP:
    device->setBaseControl(0, 0); // STOP

}

