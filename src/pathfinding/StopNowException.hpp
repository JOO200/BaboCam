//
// Created by johannes on 5/2/19.
//

#ifndef BABOCAM_STOPNOWEXCEPTION_HPP
#define BABOCAM_STOPNOWEXCEPTION_HPP


/**
 * Exception, welches während eine Strategie geworfen werden kann. Der Roboter stoppt dann auf der Stelle.
 * Der Roboter fährt danach auch nicht weiter, sondern bleibt einfach stehen.
 */
class StopNowException : public std::exception {
public:
    StopNowException() = default;
};


#endif //BABOCAM_STOPNOWEXCEPTION_HPP
