#include "../../robots/betaflight_uav.h"

int main(int argc, char *argv[]){
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyACM0";
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

	/*setup_arm_flags();

    Callbacks cbs;
    fcu::FlightController fcu(device, baudrate);
    fcu.initialise();
	
    // subscribe with custom type
    fcu.subscribe(&Callbacks::onIdent, &cbs, 1);
	fcu.subscribe(&Callbacks::onAnalog, &cbs, 5);*/

	BoBRobotics::Robots::betaflight_uav my_drone(device, baudrate);

	my_drone.subscribe();

    // Ctrl+C to quit
    std::cin.get();
}
