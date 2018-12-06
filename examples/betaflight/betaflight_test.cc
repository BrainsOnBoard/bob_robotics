// C++ includes
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <future>

#include "../../robots/betaflight_uav.h"
#include "redirect_net.h"

// BoB robotics includes
#include "../../vicon/capture_control.h"
#include "../../vicon/udp.h"



using namespace BoBRobotics::Vicon;
using namespace std::literals;

std::string GetLineFromCin() {
    std::string line;
    std::cin >> line;
    return line;
}

int main(int argc, char *argv[]){
  // can be set from the command line - default to LINUX standards
  const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
  const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

	BoBRobotics::Robots::betaflight_uav my_drone(device, baudrate);

  std::this_thread::sleep_for(1s);

	my_drone.subscribe();

	m_Port = 50091;
  m_Send_Port = 50101;

  // initialise the vicon link
  UDPClient<> vicon(51001);
  //CaptureControl viconCaptureControl("192.168.1.100", 3003, "c:\\users\\ad374\\Desktop");

	int count = 0;
	int rx_timeout = 50;

  auto future = std::async(std::launch::async, GetLineFromCin);

  bool mute = true;
  bool run = true;

  while (run) {
      if (my_drone.getArmStateAsString().size() > 0) {
          std::cout << my_drone.getArmStateAsString() << std::endl;
      }
      if (!mute) std::cout << "V = " << my_drone.getVoltage() << std::endl;

      ++count;

      if (vicon.getNumObjects() != 1) {
        // safety - VICON dropout, or too many objects, should trigger RX loss
        // note - this currently does not function as desired as dropout does not cause
        // getNumObjects to return a lower number
        rx_timeout--;

      } else {


        auto objectData = vicon.getObjectData(0);
        const auto position = objectData.getPosition<>();
        const auto attitude = objectData.getAttitude<degree_t>();
        if (!mute) std::cout << position[0] << ", " << position[1] << ", " << position[2] << ", "
                  << attitude[0] << ", " << attitude[1] << ", " << attitude[2] << std::endl;


      /*  if (count > 3000 && count < 3100) {
            my_drone.armDrone();
        }
        if (count > 20000) {
            my_drone.disarmDrone();
        }
        if (count > 6000) {
            my_drone.setVerticalSpeed(-1.0);
        }
        std::cout << count << std::endl; */

        rx_timeout = 50;
      }

      std::this_thread::sleep_for(10ms);

      if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
          auto line = future.get();

          future = std::async(std::launch::async, GetLineFromCin);

          if (line == "m") {
            mute = !mute;
          }
          if (line == "q") {
            run = false;
          }
      }

  		if (rx_timeout > 0) {
  		    my_drone.sendCommands();
  		}
    }

    // shut down threads
    exit(0);
}
