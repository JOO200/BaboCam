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
        m.unlock();   // Wir locken am Anfang den Mutex, damit keiner ohne Daten arbeitet.
        sharp_dx = NAN;
        sharp_max_dist = NAN;
    }

    std::condition_variable &getCond() {
        return wait;
    }

    std::mutex &getM() { return m; }

    ball_pos & getBall() { return curr_ball; }
    void setBall(const ball_pos pos) { curr_ball = pos; }

    double & getSharpDx() { return sharp_dx; }
    void setSharpDx(double sharpDx) { sharp_dx = sharpDx; }

    double & getSharpMaxDist() { return sharp_max_dist; }
    void setSharpMaxDist(double sharpDist) { sharp_max_dist = sharpDist; }

    State & getState() { return curr_state; }
    void setState(State state) { curr_state = state; }

private:
    State curr_state;
    ball_pos curr_ball;
    std::condition_variable wait;
    std::mutex m;
    double sharp_dx; // 0 = mittig, NaN = nicht da. Distanz zur Mitte in mm nach rechts und links
    double sharp_max_dist;  // maximaler Abstand des Balles in mm
};



#endif /* SRC_STRUCT_CONTEXT_HPP_ */
