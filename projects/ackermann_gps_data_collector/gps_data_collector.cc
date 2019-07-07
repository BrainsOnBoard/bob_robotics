// standard includes
#include<string>
#include<vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <ctime>


// BoB includes
#include "../../common/gps.h"
#include "../../common/logging.h"
#include "../../common/map_coordinate.h"

#include "imgproc/opencv_unwrap_360.h"
#include "video/panoramic.h"

// Standard C++ includes
#include <iostream>

using namespace BoBRobotics;
using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Video;


struct GpsThreadObject {
    BoBRobotics::GPS::Gps gps{ "/dev/ttyACM0" };
    std::mutex dataMutex;
    std::atomic<bool> stopFlag{ false };
    BoBRobotics::GPS::GPSData data;
};

void runGPSThread(GpsThreadObject &gpsObj) {
   
    // write gps coordinates to file 
    std::ofstream coordinates;
    coordinates.open ("coordinates.csv");
    while (!gpsObj.stopFlag) {
    
        try {
            std::lock_guard<std::mutex> lock{ gpsObj.dataMutex };
            BoBRobotics::GPS::GPSData data = gpsObj.gps.getGPSData();
            BoBRobotics::MapCoordinate::GPSCoordinate coord = data.coordinate;
            gpsObj.data = data;
            BoBRobotics::GPS::TimeStamp time = data.time;
           
            std::string latstr, lonstr;
            std::stringstream ss; 

            
            ss.precision(9);
            ss << coord.lat.value();
            latstr = ss.str();
            ss=std::stringstream();
            ss.precision(9);
            ss <<std::fixed << coord.lon.value();
            lonstr = ss.str();
            //  headers:   timestep|latitude|longitude|velocity|hour|minute|second|millisecond|
            coordinates << latstr << "," << lonstr << "," << data.velocity.value() << ",";
            coordinates << time.hour << "," << time.minute << "," <<  time.second << "," << time.millisecond << "\n";
           
        } catch(BoBRobotics::GPS::GPSError &e) { LOG_WARNING << e.what(); }
        // The gps receiver is set to 5HZ
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
    }
    coordinates.close();
}

void gpsQualityPrinter(BoBRobotics::GPS::GPSQuality quality) {
    if (quality == BoBRobotics::GPS::GPSQuality::FRTK) {
        std::cout << " GPS quality : FRTK" << std::endl;
    }
    else if(quality == BoBRobotics::GPS::GPSQuality::RTK) {
        std::cout << " GPS  quality : RTK" << std::endl;
    }
    else if(quality == BoBRobotics::GPS::GPSQuality::DGPSFIX) {
        std::cout << " GPS  quality : DGPS" << std::endl;
    }
    else if(quality == BoBRobotics::GPS::GPSQuality::GPSFIX) {
        std::cout << " GPS  quality : GPS" << std::endl;
    }
}

void setupGPS(BoBRobotics::GPS::Gps &gps) {
    
    const int maxTrials = 20;
    int numTrials = maxTrials;
    while (numTrials > 0) {
        try {
            BoBRobotics::GPS::GPSQuality quality = gps.getGPSData().gpsQuality;
            if (quality != BoBRobotics::GPS::GPSQuality::INVALID) {
                std::cout << " we have a valid measurement" << std::endl;
                gpsQualityPrinter(quality);
                break;
            }
        } catch(...) {
            std::cout << " measuring failed, trying again in 1 second " << "[" << maxTrials - numTrials << "/" << maxTrials << "]" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            numTrials--;
        }    
    }  
    if (numTrials == 0) {
        std::cout << " There is no valid gps measurement, please try waiting for the survey in to finish and restart the program " << std::endl;
        exit(1);
    }
    ///------------------------------
}

int main() {

    std::cout.precision(9);
    const int width = 450;
    const int height = 150;
    // setting up the camera ----------------------------
    const cv::Size unwrapRes(width, height);
    // Create panoramic camera and suitable unwrapper
    See3CAM_CU40 cam("/dev/video0", See3CAM_CU40::Resolution::_1280x720);
    //auto cam = getPanoramicCamera();
    auto unwrapper = cam.createUnwrapper(unwrapRes);
    const auto cameraRes = cam.getOutputSize();
    const unsigned int rawWidth = cam.getWidth() / 2;
    const unsigned int rawHeight = cam.getHeight() / 2;
    auto autoExposureMask = cam.createBubblescopeMask(cv::Size(rawWidth, rawHeight));
    // Get initial brightness and exposure
    int32_t brightness = cam.getBrightness();
    int32_t exposure = cam.getExposure();

    // Create images
    cv::Mat originalImage(cameraRes, CV_8UC3);
    cv::Mat outputImage(unwrapRes, CV_8UC3);
    //---------------------------------------------------


    std::thread gpsThread;
    GpsThreadObject *gpsObj = new GpsThreadObject;
    setupGPS(gpsObj->gps);
    
    gpsThread = std::thread(runGPSThread, std::ref(*gpsObj));
    std::cout << "gps thread started" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
   
    bool running = true;
    int frameNumber = 0;
    while (running ) {
       
        // Read from camera----------------------
        if(!cam.readFrame(originalImage)) {
            return EXIT_FAILURE;
        }
        
        time_t t = time(NULL);
        tm* timePtr = localtime(&t);
        
        int sec = (timePtr->tm_sec);
        int min = (timePtr->tm_min);
        int hour = (timePtr->tm_hour);
        std::ostringstream secs, mins, hours;
        
        if (sec < 10) { secs << 0 << sec; } else { secs << sec;}
        if (min < 10) { mins << 0 << min; } else { mins << min;}
        if (hour < 10) { hours << 0 << hour; } else { hours << hour;}   
        // Unwrap
        unwrapper.unwrap(originalImage, outputImage);
        std::ostringstream fileString;
        fileString << "image" << std::to_string(frameNumber)  << "_" << hours.str() << mins.str() << secs.str() << ".png";
        cv::imwrite(fileString.str(), outputImage);
        //-------------------------------------------


        cv::namedWindow("Unwrapped", cv::WINDOW_NORMAL);
        cv::resizeWindow("Unwrapped", width,height);
        cv::imshow("Unwrapped", outputImage);

        frameNumber++;
        const int key = cv::waitKey(1);
        if (key == 'q') {
            running = false;
        }

        if (key == 'g') {
            std::lock_guard<std::mutex> lock{ gpsObj->dataMutex };
            BoBRobotics::GPS::GPSData data = gpsObj->data;
            BoBRobotics::MapCoordinate::GPSCoordinate coord = data.coordinate;
             BoBRobotics::GPS::GPSQuality quality = data.gpsQuality;
            std::cout <<std::fixed<< "latitude: " << coord.lat.value()<< " longitude: " <<   coord.lon.value() << std::endl; 
            gpsQualityPrinter(quality);
        }
        
         else if(key == 'a') {
                cam.autoExposure(autoExposureMask);
        }
        else if(key ==  '-') {
            if(brightness > 1) {
                brightness--;
                cam.setBrightness(brightness);
                LOG_INFO << "Brightness:" << brightness;
            }
        }
        else if(key == '+') {
            if(brightness < 40) {
                brightness++;
                cam.setBrightness(brightness);
                LOG_INFO << "Brightness:" << brightness;
            }
        }
        else if(key == ',') {
            if(exposure > 1) {
                exposure--;
                cam.setExposure(exposure);
                LOG_INFO << "Exposure:" << exposure;
            }
        }
        else if(key == '.') {
            if(exposure < 9999) {
                exposure++;
                cam.setExposure(exposure);
                LOG_INFO << "Exposure:" << exposure;
            }
        }

        
    }
    
    gpsObj->stopFlag = true;
    gpsThread.join();

    return 0;

}

