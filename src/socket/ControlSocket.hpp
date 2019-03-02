/*
 * InfoSocket.hpp
 *
 *  Created on: 19.02.2019
 *      Author: johannes
 */

#ifndef SRC_BABOCAM_SOCKET_CONTROLSOCKET_HPP_
#define SRC_BABOCAM_SOCKET_CONTROLSOCKET_HPP_

#include <string>
#include <vector>
#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>

class ControlSocket {
public:
	ControlSocket(uint16_t port);
	~ControlSocket();
	int start();
	void stop();
private:
	uint16_t m_port;
	std::vector<sockaddr> m_activeConnections;
};

class ControlDataSend {
public:
	ControlDataSend();
	std::string toJson();
private:
	bool ballFound;
	double ballPosX, ballPosY;

};



#endif /* SRC_BABOCAM_SOCKET_CONTROLSOCKET_HPP_ */
