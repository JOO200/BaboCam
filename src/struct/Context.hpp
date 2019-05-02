/*
 * Context.hpp
 *
 *  Created on: Mar 4, 2019
 *      Author: johannes
 */

#ifndef SRC_STRUCT_CONTEXT_HPP_
#define SRC_STRUCT_CONTEXT_HPP_

#include <vector>
#include <condition_variable>
#include <math.h>
#include "../math/Navigator2D.hpp"

#define ball_pos Navigator2D

class Context {
public:

    enum State {
        WAIT,
        FOLLOW,
        KICK
    };

    Context() {
        m.unlock();   // Wir unlocken am Anfang den Mutex, damit keiner ohne Daten arbeitet.
        data_mutex.unlock();
        sharp_dx = NAN;
        sharp_max_dist = NAN;
        curr_state = FOLLOW;
    }

    std::condition_variable &getCond() {
        return wait;
    }

    std::mutex &getM() { return m; }

    std::mutex &getDataMutex() { return data_mutex; };

    ball_pos & getBall() { return curr_ball; }
    void setBall(const ball_pos pos) { curr_ball = pos; }

    double & getSharpDx() { return sharp_dx; }
    void setSharpDx(double sharpDx) { sharp_dx = sharpDx; }

    double & getSharpMaxDist() { return sharp_max_dist; }
    void setSharpMaxDist(double sharpDist) { sharp_max_dist = sharpDist; }

    State & getState() { return curr_state; }
    void setState(State state) { curr_state = state; }

    void stop() { m_stop.notify_all(); }
    std::condition_variable & getStop() { return m_stop; }

private:
    State curr_state;
    ball_pos curr_ball;
    std::condition_variable wait;
    std::condition_variable m_stop;
    std::mutex m;
    std::mutex data_mutex;
    double sharp_dx; // 0 = mittig, NaN = nicht da. Distanz zur Mitte in mm nach rechts und links
    double sharp_max_dist;  // maximaler Abstand des Balles in mm
};



#endif /* SRC_STRUCT_CONTEXT_HPP_ */
