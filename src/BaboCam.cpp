#include <iostream>
#include <opencv2/opencv.hpp>
#include <memory>
#include <syslog.h>
#include <unistd.h>
#include "socket/ControlSocket.hpp"
#include "color/CameraProcessing.hpp"
#include "color/BallFinder.hpp"
#include "pathfinding/PathFinder.hpp"
#include "interfaces/MovableDevice.hpp"
#include "extern/kobuki_driver/include/kobuki_driver/kobuki.hpp"
#include "sharp/SharpSocket.hpp"

using namespace std;

int main(int argc, char **argv) {

    // Setup Syslog
	setlogmask(LOG_UPTO(LOG_DEBUG)); // Alles ab Debug Loggen
	openlog("babocam", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1); // Log file öffnen. Siehe auch create_log.sh
	syslog(LOG_INFO, "Starting up.");   // Erster Log Eintrag.
    Context context;
#if 1
    // Thread zum Verarbeiten von Informationen der Sharp-Sensoren
	SharpSocket socket(&context);     // Socketthread für Laserdistanzsensoren erstellen
	socket.start();         // ... und starten
#endif
#undef ONLY_SHARP
#ifdef ONLY_SHARP
	socket.join();
#endif

#ifndef ONLY_SHARP
	rs2::frame_queue color, depth;

	syslog(LOG_INFO, "Starting Cam.");
	// Thread zum Verarbeiten der Kamerabilder starten
	CameraProcessing * processing = new CameraProcessing(color, depth);
	processing->start();
	sleep(2);

    syslog(LOG_INFO, "Starting BallFinder.");
    // Thread zum Finden eines Balles starten
	BallFinder * ballFinder = new BallFinder(processing, color, depth, &context, 0.12);
	ballFinder->start();

#define USE_KOBUKI
#ifdef USE_KOBUKI
	// Wenn wir einen Kobuki-Treiber nutzen, dann starte diesen.
    syslog(LOG_INFO, "Starting Kobuki driver.");
	kobuki::Kobuki kobuki1;
	kobuki::Parameters parameters;
	kobuki1.init(parameters);
	kobuki1.setControllerGain(0, 100*1000, 100, 2000);
    MovableDevice * movableDevice = new KobukiDevice(kobuki1);
#else
    // Ansonsten nutzen wir ein Device, welches nur loggt.
    MovableDevice * movableDevice = new MovableDevice();
#endif
    syslog(LOG_INFO, "Starting PathFinder.");
	PathFinder * path = new PathFinder(&context, movableDevice);
	path->start();

#endif // ONLY SHARP
    // sleep(600);  // 600 Sekunden schlafen, dann beenden. 10 Minuten sollte locke
    // while(true); // Endlosschleife für endlosen Spaß.

#ifdef USE_KOBUKI
    // Wir fragen beim Beenden noch den Bateriestatus ab und loggen diesen
    syslog(LOG_INFO, "Battery: %f", kobuki1.batteryStatus().capacity);
    cout << "VersionData " << kobuki1.versionInfo().firmware << std::endl;

    syslog(LOG_INFO, "Stopping.");
    kobuki1.shutdown();
#endif
    movableDevice->setMove(0, 0); // Sicher ist sicher - wir stoppen den Turtlebot
    path->stop();
    ballFinder->stop();
    usleep(50);  // Warten, bis der Turtlebot den Stop-Befehl erhalten hat.
	closelog();

	return 0;
}
