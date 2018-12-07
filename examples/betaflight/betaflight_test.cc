// C++ includes
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <future>

#include "../../robots/betaflight_uav.h"
#include "redirect_net.h"

using namespace std::literals;

#include "betaflight_vicon.h"

std::string GetLineFromCin() {
    std::string line;
    std::cin >> line;
    return line;
}

int main(int argc, char *argv[]) {
  // can be set from the command line - default to LINUX standards
  const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
  const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

	BoBRobotics::Robots::betaflight_vicon my_drone(device, baudrate);

  // limits for the VICON lab in Sheffield
  my_drone.setRoomBounds(-2.2, 2.6, -4.2, 3.4 , 0.0, 2.0);

	m_Port = 50091;
  m_Send_Port = 50101;

  auto future = std::async(std::launch::async, GetLineFromCin);

  bool mute = true;
  bool run = true;
  bool controlOn = false;

  my_drone.printStatus();

  while (run) {

    if (!mute) my_drone.printStatus();

    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto line = future.get();

        future = std::async(std::launch::async, GetLineFromCin);

        if (line == "m") {
          mute = !mute;
        }
        if (line == "a") {
          my_drone.armDrone();
        }
        if (line == "h") {
          my_drone.setWaypoint(0,0,1.0);
          controlOn = true;
        }
        if (line == "l") {
          my_drone.setWaypoint(0,0,0);
          controlOn = true;
        }
        if (line == "d") {
          my_drone.disarmDrone();
          controlOn = false;
        }
        if (line == "q") {
          run = false;
        }
    }

  	my_drone.sendCommands(controlOn);

  }

  my_drone.disarmDrone();

  // shut down threads
  exit(0);
}
