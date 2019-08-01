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

// net in variables
int sockIn;
struct sockaddr_in serveraddr;

void netInSetup() {

  int sockInPort = 50102;
  // setup the input for commands
  sockIn = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockIn < 0)
      std::cerr << "ERROR opening model input socket" << std::endl;

    int optval = 1;
    setsockopt(sockIn, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval , sizeof(int));

    bzero((char *) &serveraddr, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serveraddr.sin_port = htons((unsigned short)sockInPort);

    bind(sockIn, (struct sockaddr *) &serveraddr,  sizeof(serveraddr));

    // set non-blocking
    int flags;
    flags = fcntl(sockIn, F_GETFL, 0);
    //if (flags == -1) return false;
    flags = flags | O_NONBLOCK;
    fcntl(sockIn, F_SETFL, flags);

}

int main(int argc, char *argv[]) {
  // can be set from the command line - default to LINUX standards
  const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
  const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

  netInSetup();

	BoBRobotics::Robots::betaflight_vicon my_drone(device, baudrate);

  // limits for the VICON lab in Sheffield
  //my_drone.setRoomBounds(-2.2, 2.6, -4.2, 3.4 , 0.0, 2.0);
  // extra safe
  //my_drone.setRoomBounds(-1.8, 2.2, -2.0, 2.0, 0.0, 1.5);
  // super, mega safe
  my_drone.setRoomBounds(-1.5, 1.5, -1.5, 1.5, 0.0, 1.5);

	m_Port = 50091;
  m_Send_Port = 50101;

  auto future = std::async(std::launch::async, GetLineFromCin);

  bool mute = true;
  bool run = true;
  bool controlOn = false;
  double data[3];

  sockaddr * srcaddr;
  socklen_t * srcaddr_len;

  my_drone.printStatus();

  std::vector < std::array<double,4> > wps;

  /*wps.push_back({-1.0,1.0,0.5,-180});
  wps.push_back({-1.0,-1.0,0.5,-90});
  wps.push_back({1.0,-1.0,0.5,0});
  wps.push_back({1.0,1.0,0.5,90});
  wps.push_back({1.2,0.8,0.5,45});
  wps.push_back({0.8,1.2,0.5,45});
  wps.push_back({1.2,0.8,0.5,45});*/

  wps.push_back({-1.0,1.0,0.5,0});
  wps.push_back({-1.0,-1.0,0.5,0});
  wps.push_back({1.0,-1.0,0.5,0});
  wps.push_back({1.0,1.0,0.5,0});




  int wp_ind = 0;
  bool pathOn = false;

  while (run) {

    if (!mute) my_drone.printStatus();

    // set model outputs
    bool have = false;
    while (recvfrom(sockIn, data, sizeof(data), 0, srcaddr, srcaddr_len) != -1) {
        //my_drone.setWaypoint(data[0],data[1],0.5,data[2], false);
        std::cout << "########" << std::endl;
        std::cout << data[0] << std::endl;
        std::cout << data[1] << std::endl;
        std::cout << data[2] << std::endl;
        std::cout << "########" << std::endl;
        have = true;
    }

    if (pathOn == true) {

      if (!my_drone.hasQueuedWaypoint()) {
        my_drone.setNextWaypoint(wps[wp_ind][0],wps[wp_ind][1],wps[wp_ind][2],wps[wp_ind][3]);
        wp_ind++;
        if (wp_ind > wps.size()-1) {
          wp_ind = 0;
        }
      }

    }


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
          my_drone.setWaypoint(0,0,1.0,CONST_YAW);
          my_drone.setNextWaypoint(0,0,1.0,CONST_YAW);
          controlOn = true;
          pathOn = false;
        }
        if (line == "l") {
          my_drone.setWaypoint(0,0,0,-90);
          my_drone.setNextWaypoint(0,0,0,-90);
          controlOn = true;
          pathOn = false;
        }
        if (line == "w") {
          my_drone.setWaypoint(1.5,1.5,1.3,-90);
          my_drone.setNextWaypoint(1.5,1.5,1.3,-90);
          controlOn = true;
          pathOn = false;
        }
        if (line == "p") {
          pathOn = true;
          controlOn = true;
        }
        if (line == "d") {
          my_drone.disarmDrone();
          controlOn = false;
        }
        if (line == "m") {
          if (controlOn == true)
            my_drone.activateModel();

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
