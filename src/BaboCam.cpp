#include <iostream>
#include <opencv2/opencv.hpp>
#include <memory>
#include <syslog.h>
#include <unistd.h>
#include "socket/ControlSocket.hpp"
#include "color/ColorStreamThread.hpp"
#include "extern/kobuki_driver/include/kobuki_driver/kobuki.hpp"

using namespace std;

int main(int argc, char **argv) {

	setlogmask(LOG_UPTO(LOG_DEBUG));
	openlog("babocam", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);
	syslog(LOG_INFO, "Starting up.");

	kobuki::Kobuki kobuki1;
	kobuki::Parameters parameters;
	kobuki1.init(parameters);
	kobuki1.setControllerGain(0, 100*1000, 100, 2000);
    kobuki1.setLed(kobuki::LedNumber::Led2, kobuki::LedColour::Black);
    syslog(LOG_INFO, "Heading %f", kobuki1.getHeading());
	sleep(2);
	kobuki1.setLed(kobuki::LedNumber::Led2, kobuki::LedColour::Orange);
	kobuki1.setBaseControl(150, 0); // 10 mm/s
	sleep(5);
	syslog(LOG_INFO, "Heading %f", kobuki1.getHeading());
	kobuki1.setBaseControl(-150, 0);
    kobuki1.setLed(kobuki::LedNumber::Led2, kobuki::LedColour::Black);
    sleep(5);
    syslog(LOG_INFO, "Heading %f", kobuki1.getHeading());
    kobuki1.setBaseControl(0, 0);

#if 0
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
#endif
    sleep(5);

    syslog(LOG_INFO, "Battery: %f", kobuki1.batteryStatus().capacity);
    cout << "VersionData " << kobuki1.versionInfo().firmware << std::endl;
	closelog();

	return 0;
}
