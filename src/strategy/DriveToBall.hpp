//
// Created by johannes on 5/2/19.
//

#ifndef BABOCAM_DRIVETOBALL_HPP
#define BABOCAM_DRIVETOBALL_HPP

#include "AbstractStrategy.hpp"
#include "../extern/kobuki_driver/include/kobuki_driver/kobuki.hpp"

class DriveToBall : public AbstractStrategy {
public:
    DriveToBall() = default;
    void step(Context * curr_context, MovableDevice * kobuki, std::atomic<bool> & m_stop) override;
};


#endif //BABOCAM_DRIVETOBALL_HPP
