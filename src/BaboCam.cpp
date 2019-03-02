#include <iostream>
#include <opencv2/opencv.hpp>
#include <memory>
#include <syslog.h>
#include <unistd.h>
#include "socket/ControlSocket.hpp"
#include "color/ColorStreamThread.hpp"

using namespace std;

int main(int argc, char **argv) {

	setlogmask(LOG_UPTO(LOG_INFO));
	openlog("babocam", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);
	syslog(LOG_INFO, "Starting up.");

	ControlSocket socket = ControlSocket(4567);
	if(socket.start() < 0) {
		syslog(LOG_WARNING, "Failed");
	}

	syslog(LOG_INFO, "Finishing.");

	rs2::pipeline * cam = new rs2::pipeline();
	cam->start();

//	DepthStream stream = DepthStream();
//	stream.init(cam);
//	stream.start();

	ColorStream color = ColorStream();
	color.init(cam);
	color.start();
	color.stop();


	closelog();

	return 0;
}
