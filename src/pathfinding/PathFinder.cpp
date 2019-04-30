/*
 * PathFinder.cpp
 *
 *  Created on: Mar 4, 2019
 *      Author: johannes
 */




#include <syslog.h>
#include <math.h>
#include "PathFinder.hpp"
#include "../math/own_math.hpp"

#define MIN_ANGLE 0.2
#define MIDDLE_ANGLE 0.25

void PathFinder::run() {
    static int start_counter = 0;
    syslog(LOG_INFO, "Starting pathfinder.");
    std::unique_lock<std::mutex> lck(context->getM());

    syslog(LOG_INFO, "While-Loop");
    while(!m_stop) {
        std::cv_status returnVal = context->getCond().wait_for(lck, std::chrono::seconds(2));
        if(returnVal == std::cv_status::timeout) {
            syslog(LOG_ERR, "Timeout");
            if(device) device->setBaseControl(0, 0);
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
            speed = range(distance*100, 100, 200);
            ratio = 0;
        } else {
            ratio = 1;
            speed = context->getBall().getAngle() > 0 ? -70 : 70;
        }
        if(ratio == 1) {
            speed = absRange(speed, 25.0, 70.0);
        }
        start_counter++;
        syslog(LOG_INFO, "Distance %f, angle %f, Speed %f, ratio %f", context->getBall().getDistance(), context->getBall().getAngle(), speed, ratio);

        if(device && start_counter > 5) // wir warten etwas, bevor wir losfahren. Das Bild stabilisiert sich erst nach ein paar Messungen.
            device->setBaseControl((int16_t)speed, (int16_t)ratio);
    }
    if(device) device->setBaseControl(0, 0); // STOP
}

