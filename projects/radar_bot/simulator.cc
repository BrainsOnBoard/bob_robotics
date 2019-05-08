// Standard C++ includes
#include <fstream>
#include <limits>
#include <iostream>
#include <set>
#include <sstream>
#include <vector>
#include <chrono>
#include <thread>

// Standard C includes
#include <cassert>
#include <cstdlib>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <iterator>
#include <array>

// genn generated code
#include "Obs_net_CODE/definitions.h"

// Third-party includes
#include "third_party/units.h"

// BoB robotics includes
#include "radar/radar.h"
#include "robots/norbot.h"

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


void printDense(std::string name, scalar *g, size_t numPre, size_t numPost)
{
    std::cout << name << std::endl;
    for(size_t i = 0; i < numPre; i++) {
        for(size_t j = 0; j < numPost; j++) {
            const scalar weight = *g++;
            if (weight < 0) {
                std::cout << weight << ", ";
            } 
            else if (weight > 0 ) {
                std::cout << weight << ", ";
            } 
            else if(weight == 0) {
                std::cout << 0 << ", ";
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}


void setupConnections() { 

    // obstacle net to motor pop
    gObs_motor[0] = 0.55;
    gObs_motor[2] = 0.80;
    gObs_motor[4] = 1.75;
    gObs_motor[6] = 2.00;

    gObs_motor[9] = 2.00;
    gObs_motor[11] =1.75;
    gObs_motor[13] =0.80;
    gObs_motor[15] =0.55;

    // motor mutual inhibition
    gMotor_inh[0] =  0.0;
    gMotor_inh[1] = -4.0;
    gMotor_inh[2] = -4.0;
    gMotor_inh[3] =  0.0;

    // input inhibiting speed
    gObs_inh_speed[0] = -0.25;
    gObs_inh_speed[1] = -0.50;
    gObs_inh_speed[2] = -0.75;
    gObs_inh_speed[3] = -1.00;
    gObs_inh_speed[4] = -1.00;
    gObs_inh_speed[5] = -0.75;
    gObs_inh_speed[6] = -0.50;
    gObs_inh_speed[7] = -0.25;
}


int main()
{
    // Create thread to read radar
    std::mutex mutex;
    size_t goodFrame = -1;
    std::vector<std::tuple<double,double,double>> coords;
    std::thread thread(radarThread, std::ref(mutex), std::ref(coords), std::ref(goodFrame));

    size_t lastProcessedFrame = -1;
    // Create motor interface
    Robots::Norbot robot;

    allocateMem();
    initialize();
    
    setupConnections();
    initializeSparse();

    printDense("obs to motor", gObs_motor, 8,2);
    printDense("motor inh", gMotor_inh, 2,2);

    std::array<float, 8> binSum;
    std::array<unsigned int, 8> binCount;
    constexpr float binSize = 0.25f;

    try {
        while(true) {
            std::fill(binSum.begin(), binSum.end(), 0.0f);
            std::fill(binCount.begin(), binCount.end(), 0);

            robot.stopMoving();

            while(true) {
                auto sleepEnd = std::chrono::system_clock::now() + std::chrono::milliseconds(100);
                std::this_thread::sleep_until(sleepEnd);
                {
                    std::lock_guard<std::mutex> l(mutex);
                    
                    if(lastProcessedFrame != goodFrame) {
                        //std::cout << "output:" << std::endl;
                        for(const auto &c : coords) {
                            if(std::get<2>(c) > 0.0) {
                                std::cout << std::get<0>(c) << "," << std::get<1>(c) << "," << std::get<2>(c) << std::endl;
                                int bin =  4 + (int)std::round(std::get<0>(c) / binSize);
                                bin = std::max(0, std::min(7, bin));

                                binSum[bin] += std::get<1>(c);
                                binCount[bin]++;
                            }
                        }

                        lastProcessedFrame = goodFrame;
                        break;
                    }
                    //else {
                    //    std::cout << "ZZZZZ" << std::endl;
                   // }
                }
            }

             std::transform(binSum.cbegin(), binSum.cend(), binCount.cbegin(), binSum.begin(),
                            [](float sum, int count){ return (count == 0) ? 0.0f : std::exp(-(2.0f * sum) / (float)count); });


            // give input to the network
            std::cout << "Radar:";
            for (int j =0; j< 8;j++) {
                std::cout << binSum[j] << ",";
                ampObs_curr[j] = binSum[j];
            }
            std::cout << std::endl;
       

            //simulate 100 timestep
            std::array<int, 8> spikes;
            int speedSpikes = 0;
        
            std::fill(spikes.begin(),spikes.end(), 0);

            // simulate 100 timestep
            for (int i =0; i<100; i++) {
                ampSpeed_curr[0] = 1.5;
                stepTime();
            
                for (unsigned int s=0; s< spikeCount_Motor_pop;s++) {
                    spikes[spike_Motor_pop[s]]++;

                }
                for (unsigned int s=0; s< spikeCount_Speed_pop;s++) {
                    speedSpikes++;
                }
            }
            std::cout << " speed " << speedSpikes << std::endl;

            
            // Steer based on signal
            if(std::abs(spikes[1] - spikes[0]) <= 1) {
                robot.tank(1.0f, 1.0f);

                const auto sleepEnd = std::chrono::system_clock::now() + std::chrono::seconds(1);
                while(std::chrono::system_clock::now() < sleepEnd) {
                    std::this_thread::sleep_until(sleepEnd);
                }
            }
            else {
                if(spikes[1] > spikes[0]) {
                    robot.tank(0.25f, -0.25f);
                }
                else {
                    robot.tank(-0.25f, 0.25f);
                }

                const auto sleepEnd = std::chrono::system_clock::now() + std::chrono::milliseconds(500);
                while(std::chrono::system_clock::now() < sleepEnd) {
                    std::this_thread::sleep_until(sleepEnd);
                }

            }
           std::cout << "done moving" << std::endl;

        }
    } 
    catch(...) {
        std::cerr << "Exception" << std::endl;
        robot.stopMoving();
    }
    return 0;
}

