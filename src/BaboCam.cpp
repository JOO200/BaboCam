#include <iostream>
#include <opencv2/opencv.hpp>
#include <memory>
#include <syslog.h>
#include <unistd.h>
#include "socket/ControlSocket.hpp"
#include "color/CameraProcessing.hpp"
#include "color/BallFinder.hpp"
#include "pathfinding/PathFinder.hpp"
#include "extern/kobuki_driver/include/kobuki_driver/kobuki.hpp"
#include "sharp/SharpSocket.hpp"

using namespace std;

int main(int argc, char **argv) {

	setlogmask(LOG_UPTO(LOG_DEBUG)); // Alles ab Debug Loggen
	openlog("babocam", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1); // Log file öffnen. Siehe auch create_log.sh
	syslog(LOG_INFO, "Starting up.");   // Erster Log Eintrag.
    Context context;
#if 1
	SharpSocket socket(&context);     // Socketthread für Laserdistanzsensoren erstellen
	socket.start();         // ... und starten
#endif
#define ONLY_SHARP
#ifdef ONLY_SHARP
	socket.join();
#endif

#ifndef ONLY_SHARP
	rs2::frame_queue color, depth;

	syslog(LOG_INFO, "Starting Cam.");
	CameraProcessing * processing = new CameraProcessing(color, depth);
	processing->start();
	sleep(2);

    syslog(LOG_INFO, "Starting BallFinder.");
	BallFinder * ballFinder = new BallFinder(processing, color, depth, &context, 0.12);
	ballFinder->start();

#undef USE_KOBUKI
#ifdef USE_KOBUKI
    syslog(LOG_INFO, "Starting Kobuki driver.");
	kobuki::Kobuki kobuki1;
	kobuki::Parameters parameters;
	kobuki1.init(parameters);
	kobuki1.setControllerGain(0, 100*1000, 100, 2000);

    syslog(LOG_INFO, "Starting PathFinder.");
	PathFinder * path = new PathFinder(&context, &kobuki1);
	path->start();
#else
	syslog(LOG_WARNING, "Starting PathFinder with dummy Kobuki driver.");
	PathFinder * path = new PathFinder(&context, nullptr);
	path->start();
#endif
    sleep(500);

#ifdef USE_KOBUKI
    syslog(LOG_INFO, "Battery: %f", kobuki1.batteryStatus().capacity);
    cout << "VersionData " << kobuki1.versionInfo().firmware << std::endl;

    syslog(LOG_INFO, "Stopping.");
#endif
    path->stop();
    ballFinder->stop();

#ifdef USE_KOBUKI
    kobuki1.setBaseControl(0, 0);
#endif
#endif
    usleep(50);  // Warten, bis der Turtlebot den Stop-Befehl erhalten hat.
	closelog();

	return 0;
}
