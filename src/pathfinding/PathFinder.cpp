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
                    // Innerhalb der nächsten 2 Sekunden sollten wir ein Position von dem Ball erhalten.
                    returnVal = context->getCond().wait_for(lck, std::chrono::seconds(2));
                    if(returnVal == std::cv_status::timeout) {
                        // Wenn nicht, dann bleiben wir stehen und schreiben das in das Log
                        syslog(LOG_ERR, "Timeout");
                        device->setMove(0, 0);
                        break;
                    }
                    // Strategie zum Ball fahren ausführen
                    drive.step(context, device, m_stop);
                    break;
                case Context::State::KICK:
                    // Wir fahren noch ein Stück weiter. Einfach um sicher zu sein, dass wir von den Sensoren etwas empfangen.
                    device->setMove(100, 0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(800));
                    device->setMove(0, 0);
                    // Innerhalb der nächsten 2 Sekunden sollte über das Socket ein neuer Abstandswert empfangen werden.
                    returnVal = context->getCond().wait_for(lck, std::chrono::seconds(2));
                    if(returnVal == std::cv_status::timeout) {
                        // Wenn nicht, dann bleiben wir stehen und schreiben das in das Log
                        syslog(LOG_ERR, "Timeout");
                        device->setMove(0, 0);
                        continue;
                    }
                    kick.step(context, device, m_stop);
                    break;
                case Context::State::WAIT:
                    device->setMove(0, 0); // Bleib stehen
                    std::this_thread::sleep_for(std::chrono::seconds(10)); // und warte 10 Sekunden
                    context->setState(Context::State::FOLLOW); // Danach findest du den Ball und fähst zu ihm.
                    break;
                default:
                    syslog(LOG_WARNING, "Unknown State received. Stopping Device and wait for valid State.");
                    device->setMove(0, 0);
                    break;

            }
        } catch (StopNowException& ex) {
            // Wenn eine StopNowException geworfen wurde, bleiben wir sofort stehen.
            syslog(LOG_ERR, "Stop now!");
            break;
        }
        // syslog(LOG_INFO, "Distance %f, angle %f, Speed %f, ratio %f", context->getBall().getDistance(), context->getBall().getAngle(), speed, ratio);

    }
    if(device) device->setMove(0, 0); // STOP
}

