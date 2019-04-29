/**
 *  @example serial_port_read.cpp
 */
#include <fstream>


#include "common/timer.h"

#include "enose/enose.h"

using namespace BoBRobotics;

int main()
{
    ENose enose("/dev/ttyACM0");

    std::ofstream file("test.csv");

    Timer<> timer("");
    while(enose.update()) {
        // If we have data, write it to CSV
        if(enose.hasData()) {
            file << timer.get() << "," << enose.getGas()[0] << ", "<< enose.getGas()[1] << ", " << enose.getGas()[2] << ", " << enose.getGas()[3] << std::endl;
        }
    }

    return EXIT_SUCCESS ;
}
