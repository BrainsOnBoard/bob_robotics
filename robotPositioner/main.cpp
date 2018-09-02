// C++ includes
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <csignal>

#include "robotPositioner.h"

int main() {


    robotPositioner robp;

    std::cout << " start robot " << std::endl;
    robp.startSession("dummy_coordinates.csv");

    std::cout << " the end " << std::endl;
    return 0;
}
