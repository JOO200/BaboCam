#include <utility>

//
// Created by johannes on 4/24/19.
//

#include <netinet/in.h>
#include <syslog.h>
#include <arpa/inet.h>
#include <cstring>
#include "SharpSocket.hpp"
#include "../struct/Context.hpp"
#include <json.hpp>
#include <asio/read.hpp>

// Mappt die ADC-PINS auf die entsprechenden Positionen. Dabei ist das erste Element im Array der Sensor ganz links.
// Die Abstände sind im Array DX und die maximal möglichen Distanzen sind im Array DISTANCE zu finden
const std::string PINS[] = {
        "P9_39",
        "P9_37",
        "P9_38",
        "P9_40"
};

const double DISTANCE[] = {
        125, 280, 280, 125
};

const double DX[] = {
        -80, -40, 40, 80
};

const double VOLTAGE = 0.30; // alles über 0.3V ist ein Objekt

/**
 * Ctor SharpSocket
 * @param context verwendeter Kontext
 * @param target Verbindungsziel, z.B. "192.168.7.2" (Standard-IP für BBB per USB angeschlossen
 * @param port Port des Zielservers
 */
SharpSocket::SharpSocket(Context * context, std::string target, short port):m_context(context),m_target(std::move(target)),m_port(port) {

}

void SharpSocket::run() {
    int sock = 0;
    struct sockaddr_in serv_addr;

    char inbuffer[1024];

    static const char* message = "Hi";

    while(!m_stop) {
#if 0
        if(m_context->getBall().getDistance() > 25) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
#endif
/**
 * Hinweis: Es ist nicht das schönste, dass hier an dieser Stelle jedesmal die Socket-Verbindung komplett neu aufgebaut wird.
 * Das sollte schöner gestaltet werden, z.B. indem die Verbindung einmalig aufgebaut wird.
 * Dabei sollte natürlich auch über eine Fehlerbehandlung das automatische Neuverbinden möglich sein.
 */
        if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            syslog(LOG_ERR, "Socket failed.");
            return;
        }
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(m_port);
        if(inet_pton(AF_INET, m_target.c_str(), &serv_addr.sin_addr) <= 0) {
            syslog(LOG_ERR, "Invalid address.");
            return;
        }

        if(connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr))) {
            syslog(LOG_WARNING, "Connection failed");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        send(sock, &message, strlen(message), 0); // Wir senden "irgendwas", der Socket antwortet immer mit den erwarteten Daten
        read(sock, inbuffer,1024);
        shutdown(sock, 2);

        std::string s(inbuffer);
        if(!validate_json(s)) {
            syslog(LOG_ERR, "No valid json arrived.");
            continue;
        }

        nlohmann::json json = nlohmann::json::parse(s); // Json deserialisieren
        uint8_t countFound(0);                          // Zählervariable zum Mittelwert bilden
        double voltages[4];

        double position(NAN);                           // Die Position des Balles nach links und rechts in Millimeter
        double maxDist(NAN);
        for(uint8_t i = 0; i < 4; ++i) {                // alle 4 Sensoren
            voltages[i] = json.at(PINS[i]);             // Die Spannung wird vom ADC-Pin auf das Array gemappt
            //voltages[i] = json[PINS[i]];
            if(voltages[i] > VOLTAGE) {                 // Wenn die Spannung über den Grenzwert ist, ist hier ein Objekt
                position = (isnan(position)) ? DX[i] : (position * countFound + DX[i])/(countFound+1); // Durchschnitt
                maxDist = std::max(maxDist, DISTANCE[i]);
                countFound++;
            }
        }

        syslog(LOG_INFO, "Voltages[%f, %f, %f, %f]", voltages[0], voltages[1], voltages[2], voltages[3]);
        syslog(LOG_INFO, "Position: %f", position);

        m_context->getDataMutex().lock(); // Daten in Kontext packen
        m_context->setSharpDx(position);
        m_context->setSharpMaxDist(maxDist);
        Context::State curr_state = m_context->getState();
        m_context->getDataMutex().unlock();
        if(curr_state == Context::State::KICK) {
            m_context->getCond().notify_all();  // Im status "Kick" weckt der Sharp-Sensor den nächsten Schritt
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(500));    // Erst nach 500ms fragen wir das nächste mal ab.
    }
}

bool SharpSocket::validate_json(std::string &incoming) {
    std::string::size_type pos = incoming.find('}');
    if(pos == std::string::npos) // Bei std::string::npos gibt es keine geschweifte Klammer - das darf es aber nicht geben.
        return false;
    // Irgendwie gibt es einen komischen Bug:
    // Der Python Socket-Server verschickt hinter dem serialisierten Json-String noch ungültige Chars
    // Die einfachste Möglichkeit war da, diese ungültigen Zeichen heraus zu löschen.
    // TODO: Json-String besser aufräumen. Aktuell funktioniert das nur, weil der String nur ein "}" enthält.
    // Wenn hier mehrere geschlossene geschweifte Klammern vorhanden sind, wird incoming.find('}') irgendwas mittem im String finden.
    incoming = incoming.substr(0, pos+1);
    return true;
}
