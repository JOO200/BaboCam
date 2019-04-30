//
// Created by johannes on 27.03.19.
//

#ifndef BABOCAM_IRUNNABLE_HPP
#define BABOCAM_IRUNNABLE_HPP

#include <atomic>
#include <thread>

/**
 *  Dieses Interface ist eine abstrakte Klasse, welche das erstellen von Threads vereinfachen soll.
 *  Dabei können neue Klassen erstellt werden und von dieser Klasse erben.
 *  Die Klassen müssen eine Methode "run" besitzen, welche die Methode überschreibt.
 *  Sollte m_stop gesetzt sein, muss die run-Methode verlassen werden.
 *
 *  Im Destruktor wird automatisch stop() aufgerufen, damit der Thread gestoppt wird.
 *
 *  Beispiel:
 *
 *  class Example : public IRunnable {
 *  public:
 *      Example(context * example):m_example(example) { }
 *  protected:
 *      override void run() {
 *          while(m_stop) {
 *              m_example->doSmth();
 *          }
 *      }
 *  private:
 *      context * m_example;
 *
 *  Nutzung:
 *      Example * ex = new Example(ex);
 *      ex.start();
 *      ...
 *      ex.stop();
 */
class IRunnable {
public:
    IRunnable():m_stop(false) {    }    // Default ctor
    virtual ~IRunnable() { stop(); };   // Default dtor

    virtual void start() { m_thread = std::thread(&IRunnable::run, this); m_stop = false; };    // Thread start
    void join() { m_thread.join(); };   // Thread join
    void stop() { if(!m_stop) { m_stop = true; m_thread.join(); }}; // Stoppen, wenn notwendig

protected:
    std::atomic<bool> m_stop;       // Variable stoppen
    virtual void run() = 0;         // Run-Methode
private:
    std::thread m_thread;

};
#endif //BABOCAM_IRUNNABLE_HPP
