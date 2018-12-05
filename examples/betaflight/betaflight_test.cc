#include "../../robots/betaflight_uav.h"

#include "redirect_net.h"

int main(int argc, char *argv[]){
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyS3";
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

	BoBRobotics::Robots::betaflight_uav my_drone(device, baudrate);

	my_drone.subscribe();
	
	m_Port = 50091;
    m_Send_Port = 50101;
	
	// set up the networking code needed to receive images:
	//if (!setupSockets()) {
	//	// error
	//	return -1;
	//}
	
	int count = 0;
	int rx_timeout = 200;

    while (1) {
        if (my_drone.getArmStateAsString().size() > 0) {
            std::cout << my_drone.getArmStateAsString() << std::endl;
        }
        std::cout << "V = " << my_drone.getVoltage() << std::endl;
        
        ++count;
        if (count > 3000 && count < 3100) {
            my_drone.armDrone();
        }
        if (count > 20000) {
            my_drone.disarmDrone();
        }
        if (count > 6000) {
            my_drone.setVerticalSpeed(-1.0);
        }
        std::cout << count << std::endl; 
        
        //readFrame();
		cmds commands = getMotorCommands();
		if( commands.have_new) {
		    //std::cout << "here" << std::endl;
		    my_drone.setRoll(commands.roll);
		    my_drone.setPitch(commands.pitch);
		    my_drone.setVerticalSpeed(commands.throttle);
		    my_drone.setYawSpeed(commands.yaw);
		 //robot.omni2D((float) commands.fb, (float) commands.lr, (float) commands.yaw);
		} else {
		    //--rx_timeout;
		}
		if (rx_timeout > 0) {
		    my_drone.sendCommands();
		}
    }

    // Ctrl+C to quit
    std::cin.get();
}
