//
// Created by johannes on 5/2/19.
//

#ifndef BABOCAM_KICKBALL_HPP
#define BABOCAM_KICKBALL_HPP


#include "AbstractStrategy.hpp"

class KickBall : public AbstractStrategy {
public:
    KickBall() = default;
    void step(Context * context, MovableDevice * kobuki, std::atomic<bool>& m_stop) override ;
};


#endif //BABOCAM_KICKBALL_HPP
