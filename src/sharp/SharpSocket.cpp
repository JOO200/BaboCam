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
        send(sock, &message, strlen(message), 0);
        read(sock, inbuffer,1024);
        shutdown(sock, 2);

        std::string s(inbuffer);
        if(!validate_json(s)) {
            syslog(LOG_ERR, "No valid json arrived.");
            continue;
        }

        nlohmann::json json = nlohmann::json::parse(s);
        uint8_t countFound(0);
        double voltages[4];

        double position(NAN);
        double maxDist(NAN);
        for(uint8_t i = 0; i < 4; ++i) {
            voltages[i] = json.at(PINS[i]);
            //voltages[i] = json[PINS[i]];
            if(voltages[i] > VOLTAGE) {
                position = (isnan(position)) ? DX[i] : (position * countFound + DX[i])/(countFound+1); // Durchschnitt
                maxDist = std::max(maxDist, DISTANCE[i]);
                countFound++;
            }
        }

        syslog(LOG_INFO, "Voltages[%f, %f, %f, %f]", voltages[0], voltages[1], voltages[2], voltages[3]);
        syslog(LOG_INFO, "Position: %f", position);

        m_context->getM().lock();
        m_context->setSharpDx(position);
        m_context->setSharpMaxDist(maxDist);
        m_context->getM().unlock();


        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

bool SharpSocket::validate_json(std::string &incoming) {
    std::string::size_type pos = incoming.find('}');
    if(pos == std::string::npos)
        return false;
    incoming = incoming.substr(0, pos+1);
    return true;
}
