#pragma once

// standar includes
#include <iostream>
#include <fstream>

#include "../../robots/norbot2.h"
#include "../../vicon/udp.h"

// Third-party includes
#include "../../third_party/units.h"

using namespace BoBRobotics;
using namespace units::length;
using namespace units::literals;
using namespace units::angle;
using namespace units::angular_velocity;
using namespace units::velocity;
using namespace std::literals;

// experiments 

void exp1(std::ofstream &file, Norbot2 &bot, Vicon::UDPClient<> &vicon) {
    // u -> 0-max speed
    file << "t (ms) , u, x (mm), y (mm) \n";
    
    for (int i = 0; i < 128; i++) {
        bot.move(i ,90); // move straight
        const auto objectData = vicon.getObjectData(0);
        const Vector3<millimeter_t> position = objectData.getPosition();
        //const Vector3<radian_t> attitude = objectData.getAttitude();
        millimeter_t posx = position[0];
        millimeter_t posy = position[1];
        //auto yaw = attitude[0];
        
        file << i*50, i , posx.value(), posy.value(), "\n";
        
        std::this_thread::sleep_for(50ms);
    }
}

void test(Norbot2 &bot) {
    for (int i = 0; i < 128; i++) {
        bot.move(i ,90); // move straight
        std::this_thread::sleep_for(50ms);
    }
}



int main() {

    Norbot2 bot;
    test(bot);
    
    /*
    
    // Connect to Vicon system
    Vicon::UDPClient<> vicon(51001);
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }
    
    Norbot2 bot;
       
    std::ofstream myfile;
    myfile.open ("exp1.csv");
    
    exp1(myfile, bot, vicon); // perform experiment 1 
    
    myfile.close();
    */
    
 
    return 0;
}




