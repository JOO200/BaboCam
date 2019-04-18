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
#include "../math/Navigator2D.hpp"

#define ball_pos Navigator2D

class Context {
public:
    Context() {
        m.lock();   // Wir locken am Anfang den Mutex, damit keiner ohne Daten arbeitet.
    }

    std::condition_variable &getCond() {
        return wait;
    }

    std::mutex &getM() {
        return m;
    }

    ball_pos & getBall() {
        return curr_ball;
    }

    void setBall(const ball_pos pos) {
        curr_ball = pos;
    }


private:
    ball_pos curr_ball;
    std::condition_variable wait;
    std::mutex m;
};



#endif /* SRC_STRUCT_CONTEXT_HPP_ */
