// BoB robotics includes
#include "net/spineml_net.h"
#include "robots/mecanum.h"
#include "video/rpi_cam.h"

// Standard C includes
#include <csignal>

int keep_running = 1;
static void signal_handler(int)
{
    keep_running = 0;
}

int main()
{

    struct sigaction sig_action = {};
    sig_action.sa_handler = signal_handler;

    // first register handler for system interrupts so we can go down gracefully on 'nix
    sigaction(SIGINT, &sig_action, NULL);
    sigaction(SIGPIPE, &sig_action, NULL);

    // Create motor interface
    BoBRobotics::Robots::Mecanum robot;

	// Create camera interface
	BoBRobotics::Video::RPiCamera cam(50091);

	// SpineML network
	BoBRobotics::Net::SpineML_Network sml_net(50100);

	// add robot and camera to network
	sml_net.setInput(cam);
	sml_net.setRobot(robot);

	// listen for connections
	sml_net.runInBackground();

	while (!sml_net.haveAllConnections()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	std::cout << "Have all connections!" << std::endl;

	// listen complete
	sml_net.stop();

	std::cout << "Running..." << std::endl;

	// start model connection
	sml_net.runConnections();

	while(keep_running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	sml_net.stopConnections();

	std::cout << "Closing..." << std::endl;

    return 0;
}
