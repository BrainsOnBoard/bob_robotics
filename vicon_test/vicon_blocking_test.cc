// C++ includes
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// BoB robotics includes
#include "../vicon/udp.h"

using namespace BoBRobotics::Vicon;

using ObjectDataType = ObjectData<millimeter_t, degree_t>;

void readCallback(uint, ObjectDataType &data, void*)
{
    const auto &position = data.getPosition();
    const auto &attitude = data.getAttitude();

    std::cout << position[0] << ", " << position[1] << ", "
              << position[2] << ", " << attitude[0] << ", " << attitude[1]
              << ", " << attitude[2] << std::endl;
}

int main()
{
    // connect to Vicon system
    UDPClient<ObjectDataType> vicon(51001);
    std::cout << "Connected to Vicon system" << std::endl;

    // set function to call on new position
    vicon.setReadCallback(readCallback, nullptr);

    // wait for keypress + return
    char c;
    std::cin >> c;

    return EXIT_SUCCESS;
}
