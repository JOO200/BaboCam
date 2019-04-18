//
// Created by johannes on 4/17/19.
//

#ifndef BABOCAM_NAVIGATOR2D_HPP
#define BABOCAM_NAVIGATOR2D_HPP


class Navigator2D {
public:
    Navigator2D(double distance, float angle):m_distance(distance),m_angle(angle) {};
    Navigator2D():m_distance(0),m_angle(0) {};

    Navigator2D(const Navigator2D & old):m_distance(old.m_distance),m_angle(old.m_angle) {} ;

    const double & getDistance() { return m_distance; }
    const float & getAngle() { return m_angle; }
private:
    double m_distance;
    float m_angle;
};


#endif //BABOCAM_NAVIGATOR2D_HPP
