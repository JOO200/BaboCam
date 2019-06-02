//
// Created by johannes on 5/2/19.
//

#include "KickBall.hpp"
#include "../pathfinding/StopNowException.hpp"

void KickBall::step(Context *context, MovableDevice *kobuki, std::atomic<bool>& m_stop) {
    context->getDataMutex().lock();

    static uint8_t counter = 0;
    if(isnan(context->getSharpDx())/* || isnan(context->getSharpMaxDist())*/) {
        syslog(LOG_ERR, "Invalid Sharp DX found.");
        kobuki->setMove(0, 0);
        counter++;
        if(counter > 10) {
            syslog(LOG_ERR, "Deadlock found. Shutting down.");
            throw StopNowException();
        }
        context->getDataMutex().unlock();
        return;
    }
    kobuki->setMove(100, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    kobuki->setMove(500, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    kobuki->setMove(0, 0);
    context->setState(Context::State::WAIT);
    context->getDataMutex().unlock();
}
