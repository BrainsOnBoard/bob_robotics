/**
 *  @example serial_port_read.cpp
 */
#include <fstream>

#include <chrono>
#include <thread>
#include <mutex>


//#include "common/timer.h"

#include "radar/radar.h"

using namespace BoBRobotics;

void radarThread(std::mutex &mutex, std::vector<std::tuple<double,double,double>> &coords, size_t &goodFrame)
{
    Radar radar("/dev/ttyACM1");
    while(radar.update()) {
        if(radar.hasData()) {
          std::lock_guard<std::mutex> l(mutex);
          radar.getObjCoords(coords);
          goodFrame = radar.getGoodFrame();
        }
    }
}
int main()
{
    using namespace std::chrono_literals;

    std::mutex mutex;
    std::vector<std::tuple<double,double,double>> coords;
    size_t goodFrame = -1;
    std::thread thread(radarThread, std::ref(mutex), std::ref(coords), std::ref(goodFrame));

    size_t lastProcessedFrame = -1;
    while(true) {
       std::this_thread::sleep_for(1.0s);

       {
         std::lock_guard<std::mutex> l(mutex);

         if(lastProcessedFrame != goodFrame) {
             std::cout << "output:" << std::endl;
             for(const auto &c : coords) {
                 std::cout << "\t" << ":" << std::get<0>(c) << "," << std::get<1>(c) << "," << std::get<2>(c) << std::endl;
             }
             lastProcessedFrame = goodFrame;
         }
       }
    }
    return EXIT_SUCCESS ;
}
