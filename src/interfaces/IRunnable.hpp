//
// Created by johannes on 27.03.19.
//

#ifndef BABOCAM_IRUNNABLE_HPP
#define BABOCAM_IRUNNABLE_HPP

#include <atomic>
#include <thread>

class IRunnable {
public:
    IRunnable():m_stop(false) {    }
    virtual ~IRunnable() { stop(); };

    virtual void start() { m_thread = std::thread(&IRunnable::run, this); m_stop = false; };
    void join() { m_thread.join(); };
    void stop() { if(!m_stop) { m_stop = true; m_thread.join(); }};

protected:
    std::atomic<bool> m_stop;
    virtual void run() = 0;
private:
    std::thread m_thread;

};
#endif //BABOCAM_IRUNNABLE_HPP
