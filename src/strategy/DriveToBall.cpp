//
// Created by johannes on 5/2/19.
//

#include <syslog.h>
#include "DriveToBall.hpp"
#include "../math/own_math.hpp"

#define MIN_ANGLE 0.2

void DriveToBall::step(Context *context, MovableDevice * kobuki, std::atomic<bool>& m_stop) {
    context->getDataMutex().lock();
    double speed(NAN), ratio(NAN); // Variablen für die Geschwindigkeit und die Drehbewegung

    if(context->getBall().getAngle() == 0 && context->getBall().getDistance() == 0) {
        // Wir wurden mit einem ungültigen Kontext aufgerufen - das kann so nicht stimmen.
        // Daher loggen wir das einmal und lassen den Roboter stehen.
        syslog(LOG_ERR, "Invalid context found.");
        kobuki->setMove(0, 0);
        context->getDataMutex().unlock();
        return;
    }
    static uint8_t start_counter = 0;
    start_counter++;
    if(start_counter < 5) {
        // Wir warten 5 Bilder ab, bis sich die Kamera "stabilisiert" hat.
        // Die ersten Bilder sind für gewöhnlich nicht schön, weil Filter und Fokus sich erst einstellen müssen.
        kobuki->setMove(0, 0);
        context->getDataMutex().unlock();
        return;
    }
    if(std::abs(context->getBall().getAngle()) < MIN_ANGLE) {
        // Wenn der relative Winkel zwischen "Geradeaus" fahren und Ball kleiner als der minimale Winkel ist, dann fahren wir geradeaus
        double distance = context->getBall().getDistance();
        speed = range(distance*100, 100, 200); // Die Geschwindigkeit ist abhängig von der Entfernung, mindestens aber 100, maximal 200.
        ratio = 0; // gerade aus fahren, keine Drehbewegung
    } else {
        // Ansonsten drehen wir uns um uns selbst.
        ratio = 1; // Drehen
        speed = context->getBall().getAngle() > 0 ? -70 : 70; // Geschwindigkeit des Drehens
    }

    if(context->getBall().getDistance() < 0.25) {
        // Wenn wir vor dem Ball stehen, dann kicken wir den Ball einmal.
        context->setState(Context::State::KICK);
    }
    context->getDataMutex().unlock(); // Data Mutex entsperren
    kobuki->setMove(speed, ratio); // Dem Device die ermittelten Werte übermitteln
}
