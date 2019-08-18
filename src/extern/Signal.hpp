//
// Created by johannes on 4/10/19.
//

#ifndef BABOCAM_SIGNAL_HPP
#define BABOCAM_SIGNAL_HPP


#include <functional>

/**
 * Generische Klasse für Signale. Jedes Signal beinhaltet eine bestimmte Nachricht.
 * Diese Klasse dient als Hilfsklasse, um Signale variabel über Callback-Methoden behandeln zu können.
 * @tparam R Inhalt der Nachricht.
 */
template <typename R>
class Signal {
public:
    Signal() = default;
    ~Signal() = default;

    /**
     * Setzt den Callback für das Signal
     * @param function Callback
     */
    void setFunction(const std::function<void(const R*)> function) {
        processing_function = function;
    }

    /**
     * Nutzt die Daten und sendet, wenn eine Funktion per Setter gesetzt wurde, diese an die Funktion, welche dafür registriert wurde.
     * @param data
     */
    void process(const R & data) {
        if(processing_function)
            processing_function(&data);
    }

private:
    std::function<void(const R*)> processing_function;
};

/**
 * Das gleiche nochmal für eine Nachricht, die keinen Inhalt hat. Nützlich für Zustandsmeldungen etc.
 */
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
