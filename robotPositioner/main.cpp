// C++ includes
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <csignal>

#include "robotPositioner.h"

// a small example program demonstrating the usage of 'robotPositioner.h'
int main() {

    unsigned int vicon_udp_client_port = 51001;                 
    std::string vicon_capture_control_ip_address = "192.168.1.100";  
    std::string vicon_capture_control_executable_path = "c:\\users\\ad374\\Desktop";   
    unsigned int capture_control_port = 3003;
    double robot_r = 6.8;                                   
    double robot_D = 10.4;
    double threshold_distance; // in mm
    double stopping_distance;  // in mm
    std::string video_device = "/dev/video0";                  
    double k1;                                              
    double k2;                                              
    double alpha;                                           
    double beta;                                            
    double max_velocity;                                   

    std::cout << "enter the allowed error on distance (stopping_distance) " << std::endl;
    std::cin >> stopping_distance;
    std::cout << "enter threshold_distance (distance where robot starts to slow down)" << std::endl;
    std::cin >> threshold_distance;
    std::cout << " enter k1 for the positioner [How curvy is the curve?]" << std::endl;
    std::cin >> k1;
    std::cout << " enter k2 for the positioner [Speed of steering]" << std::endl;
    std::cin >> k2;
    std::cout << " enter alpha " << std::endl;
    std::cin >> alpha;
    std::cout << " enter beta " << std::endl;
    std::cin >> beta;
    std::cout << " max velocity " << std::endl;
    std::cin >> max_velocity;



    robotPositioner robp(
        vicon_udp_client_port,                     
	    vicon_capture_control_ip_address,
        vicon_capture_control_executable_path,
        capture_control_port,
        robot_r,
        robot_D,
        threshold_distance, 
        stopping_distance,
        video_device,
        k1,
        k2,
        alpha,
        beta,
        max_velocity
    );

    std::cout << " start robot " << std::endl;
    robp.startSession("dummy_coordinates.csv");

    std::cout << " the end " << std::endl;
    return 0;
}
