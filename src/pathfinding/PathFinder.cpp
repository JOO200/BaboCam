/*
 * PathFinder.cpp
 *
 *  Created on: Mar 4, 2019
 *      Author: johannes
 */




#include <syslog.h>
#include <math.h>
#include "PathFinder.hpp"

#define MIN_ANGLE 50
#define MIDDLE_ANGLE 150

void PathFinder::run() {
    static int start_counter = 0;
    syslog(LOG_INFO, "Starting pathfinder.");
    std::unique_lock<std::mutex> lck(context->getM());

    syslog(LOG_INFO, "While-Loop");
    while(!m_stop) {
        std::cv_status returnVal = context->getCond().wait_for(lck, std::chrono::seconds(2));
        if(returnVal == std::cv_status::timeout) {
            syslog(LOG_ERR, "Timeout");
            device->setBaseControl(0, 0);
            continue;
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
            speed = std::min(distance*100, 200.0);
            speed = std::max(speed, 100.0);
            ratio = 0;
        } else if(std::abs(context->getBall().getAngle()) < MIDDLE_ANGLE) {
            ratio = 1;
            speed = context->getBall().getAngle()*0.05;
        } else {
            float angle = context->getBall().getAngle();
            ratio = 1;
            speed = angle * 0.1;
        }
        if(ratio == 1) {
            if(speed > 0) {
                speed = std::min(50.0, speed);
                speed = std::max(25.0, speed);
            } else {
                speed = std::max(-50.0, speed);
                speed = std::min(-25.0, speed);
            }
        }
        start_counter++;
        syslog(LOG_INFO, "Distance %f, angle %f, Speed %f, ratio %f", context->getBall().getDistance(), context->getBall().getAngle(), speed, ratio);
        if(start_counter > 10)
            device->setBaseControl((int16_t)speed, (int16_t)ratio);
    }
    device->setBaseControl(0, 0); // STOP

}

