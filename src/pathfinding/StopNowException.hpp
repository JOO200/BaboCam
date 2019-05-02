//
// Created by johannes on 5/2/19.
//

#ifndef BABOCAM_STOPNOWEXCEPTION_HPP
#define BABOCAM_STOPNOWEXCEPTION_HPP


class StopNowException : public std::exception {
public:
    StopNowException() = default;
};


#endif //BABOCAM_STOPNOWEXCEPTION_HPP
