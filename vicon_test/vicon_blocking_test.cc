// C++ includes
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// BoB robotics includes
#include "../vicon/udp.h"

using namespace BoBRobotics::Vicon;

UDPClient<ObjectData> *vicon;

void readCallback(uint, const ObjectData &data, void*)
{
    const auto &translation = data.getTranslation();
    const auto &rotation = data.getRotation();

    std::cout << translation[0] << ", " << translation[1] << ", "
              << translation[2] << ", " << rotation[0] << ", " << rotation[1]
              << ", " << rotation[2] << std::endl;
}

int main()
{
    // connect to Vicon system
    vicon = new UDPClient<ObjectData>(51001);
    std::cout << "Connected to Vicon system" << std::endl;

    // set function to call on new position
    vicon->setReadCallback(readCallback, nullptr);

    // wait for keypress + return
    char c;
    std::cin >> c;

    delete vicon;
    return EXIT_SUCCESS;
}
