//
// Created by johannes on 4/10/19.
//

#ifndef BABOCAM_SIGNAL_HPP
#define BABOCAM_SIGNAL_HPP


#include <functional>

template <typename R>
class Signal {
public:
    Signal() = default;
    ~Signal() = default;

    void setFunction(const std::function<void(const R*)> function) {
        processing_function = function;
    }

    void process(const R & data) {
        if(processing_function)
            processing_function(&data);
    }

private:
    std::function<void(const R*)> processing_function;
};

class VoidSignal {
public:
    VoidSignal() = default;
    ~VoidSignal() = default;

    void setFunction(const std::function<void(void)> function) {
        processing_function = function;
    }

    void process() {
        if(processing_function)
            processing_function();
    }

private:
    std::function<void(void)> processing_function;
};
#endif //BABOCAM_SIGNAL_HPP
