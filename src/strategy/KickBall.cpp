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
            // Wir sollten innerhalb von 10 Zyklen einen Abstand finden, sonst werden wir auch länger keinen Abstand finden
            syslog(LOG_ERR, "Deadlock found. Shutting down.");
            throw StopNowException();
        }
        context->getDataMutex().unlock();
        return;
    }
    // Wir überbrücken die Distanz bis zum Ball langsam. Dabei sind 2500ms ein Wert, der durch Tests ermittelt wurde
    kobuki->setMove(100, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    // Einmal kurz Vollgas geben kickt den Ball
    kobuki->setMove(500, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // Dann stehen bleiben.
    kobuki->setMove(0, 0);
    context->setState(Context::State::WAIT); // Wir warten erstmal, bis der Ball wieder an einem festen Punkt liegt
    context->getDataMutex().unlock();
}
