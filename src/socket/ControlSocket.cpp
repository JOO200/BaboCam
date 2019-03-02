/*
 * InfoSocket.cpp
 *
 *  Created on: 19.02.2019
 *      Author: johannes
 */
#include "ControlSocket.hpp"
#include <syslog.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <unistd.h>

ControlSocket::ControlSocket(uint16_t port) {
	m_port = port;
}

ControlSocket::~ControlSocket() {
	if(m_activeConnections.size() > 0) stop();
}

int ControlSocket::start() {
    std::cout << "Setting up Socket." << std::endl;
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if(sockfd < 0) {
		syslog(LOG_ERR, "Cannot setup ControlSocket. SockFd returned %d", sockfd);
		return -1;
	}
	struct sockaddr_in localAddr;

	localAddr.sin_family = AF_INET;
	localAddr.sin_port = htons(m_port);
	localAddr.sin_addr.s_addr = INADDR_ANY;

	if( bind(sockfd, (sockaddr*)&localAddr, sizeof(localAddr)) < 0 ) {
		syslog(LOG_ERR, "Bind failed for port %d", m_port);
		return -1;
	}

	if( listen(sockfd, 3) < 0) {
		syslog(LOG_ERR, "Failed to listen to port %d", m_port);
		return -1;
	}

	int new_socket;
	if((new_socket = accept(sockfd, (struct sockaddr*)&localAddr, (socklen_t*)sizeof(localAddr))) < 0) {
		syslog(LOG_ERR, "Failed to accept socket.");
		return -1;
	}
	char buffer[1024] = {0};
	int valread;
	valread = read( new_socket, (void*)buffer, (size_t)1024);
	syslog(LOG_INFO, "Socket started %s", buffer);

	std::cout << "Success " << std::endl;



	return 0;
}

void ControlSocket::stop() {
	
}

/*
 * ControlDataSend
 */
ControlDataSend::ControlDataSend() {
	ballFound = false;
	ballPosX = 0;
	ballPosY = 0;
}

std::string ControlDataSend::toJson() {
	//CJSON
	return "";
}


