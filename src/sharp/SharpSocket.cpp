#include <utility>

//
// Created by johannes on 4/24/19.
//

#include <netinet/in.h>
#include <syslog.h>
#include <arpa/inet.h>
#include <cstring>
#include "SharpSocket.hpp"
#include <json.hpp>
#include <asio/read.hpp>

SharpSocket::SharpSocket(std::string target, short port):m_target(std::move(target)),m_port(port) {

}

void SharpSocket::run() {
    struct sockaddr_in address;
    int sock = 0, valread;
    struct sockaddr_in serv_addr;

    char inbuffer[1024];

    static const char* message = "Hi";

    while(!m_stop) {
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
        syslog(LOG_INFO, "Send message");
        read(sock, inbuffer,1024);
        shutdown(sock, 2);

        std::string s(inbuffer);
        if(!validate_json(s)) {
            syslog(LOG_ERR, "No valid json arrived.");
            continue;
        }
        syslog(LOG_INFO, "Message received %s", inbuffer);

        nlohmann::json json = nlohmann::json::parse(s);
        double value9_37 = json["P9_37"];
        double value9_38 = json["P9_38"];
        double value9_39 = json["P9_39"];
        double value9_40 = json["P9_40"];
        syslog(LOG_INFO, "Deserialized message: %f, %f, %f, %f", value9_37, value9_38, value9_39, value9_40);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

bool SharpSocket::validate_json(std::string &incoming) {
    std::string::size_type pos = incoming.find('}');
    if(pos == std::string::npos)
        return false;
    incoming = incoming.substr(0, pos+1);
    return true;
}
