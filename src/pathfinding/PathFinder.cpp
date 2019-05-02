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
#include "StopNowException.hpp"

#define MIN_ANGLE 0.2

void PathFinder::run() {
    syslog(LOG_INFO, "Starting pathfinder.");
    std::unique_lock<std::mutex> lck(context->getM());

    syslog(LOG_INFO, "While-Loop");
    std::cv_status returnVal;
    while(!m_stop) {
        try {
            switch (context->getState()) {
                case Context::State::FOLLOW:
                    returnVal = context->getCond().wait_for(lck, std::chrono::seconds(2));
                    if(returnVal == std::cv_status::timeout) {
                        syslog(LOG_ERR, "Timeout");
                        device->setMove(0, 0);
                        break;
                    }
                    drive.step(context, device, m_stop);
                    break;
                case Context::State::KICK:
                    device->setMove(100, 0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(800));
                    device->setMove(0, 0);
                    returnVal = context->getCond().wait_for(lck, std::chrono::seconds(2));
                    if(returnVal == std::cv_status::timeout) {
                        syslog(LOG_ERR, "Timeout");
                        device->setMove(0, 0);
                        continue;
                    }
                    kick.step(context, device, m_stop);
                    break;
                case Context::State::WAIT:
                    device->setMove(0, 0);
                    std::this_thread::sleep_for(std::chrono::seconds(30));
                    context->setState(Context::State::FOLLOW);
                    break;
                default:
                    syslog(LOG_WARNING, "Unknown State received. Stopping Device and wait for valid State.");
                    device->setMove(0, 0);
                    break;

            }
        } catch (StopNowException& ex) {
            syslog(LOG_ERR, "Stop now!");
            break;
        }
        // syslog(LOG_INFO, "Distance %f, angle %f, Speed %f, ratio %f", context->getBall().getDistance(), context->getBall().getAngle(), speed, ratio);

    }
    if(device) device->setMove(0, 0); // STOP
}

