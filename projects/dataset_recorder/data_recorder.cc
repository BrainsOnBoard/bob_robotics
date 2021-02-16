// standard includes
#include<string>
#include<vector>
#include <thread>
#include <mutex>
#include <atomic>

// BoB includes
#include "common/main.h"
#include "common/stopwatch.h"
#include "common/background_exception_catcher.h"
#include "common/gps.h"
#include "common/map_coordinate.h"
#include "common/bn055_imu.h"
#include "robots/rc_car_bot.h"

#include "imgproc/opencv_unwrap_360.h"
#include "video/panoramic.h"

#include <sys/stat.h>
#include <ctime>






// Standard C++ includes
#include <iostream>

using namespace BoBRobotics;
using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Video;


#define WIDTH 192
#define HEIGHT 72

#define IS_UNWRAP true
#define UPDATE_INTERVAL 1 // 200


void readCameraThreadFunc(BoBRobotics::Video::Input *cam,
                        cv::Mat &img,
                        std::atomic<bool> &shouldQuit,
                        std::mutex &mutex)
{

    //auto unwrapper = cam->createUnwrapper(cv::Size(WIDTH,HEIGHT));

    while(!shouldQuit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::lock_guard<std::mutex> lock(mutex);
        cam->readFrame(img);


    }


}

void saveThread(cv::Mat img, std::string fileName) {

    cv::imwrite(fileName, img);
}

int bobMain(int argc, char* argv[])
{

    using degree_t = units::angle::degree_t;
    std::vector<cv::Mat> images;
    std::vector<std::string> fileNames;

    // setting up
    const char *path_linux = "/dev/ttyACM1"; // path for linux systems
    BoBRobotics::Robots::RCCarBot bot;
    BoBRobotics::GPS::Gps gps;
    gps.connect(path_linux);
    BoBRobotics::BN055 imu;
    const int maxTrials = 3;
    int numTrials = maxTrials;
    std::atomic<bool> shouldQuit{false};
    std::mutex mex;


    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%d-%m-%Y",timeinfo);
    std::string str(buffer);

    int seconds_to_run = 30; // default value
    if (argc > 1) {
        seconds_to_run = stoi(std::string(argv[1]));
        std::cout << "running for " << seconds_to_run << " seconds" << std::endl;

    }



    // check to see if we get valid coordinates by checking GPS quality
    while (numTrials > 0) {
        try {
            if (gps.getGPSData().gpsQuality != BoBRobotics::GPS::GPSQuality::INVALID) {
                std::cout << " we have a valid measurement" << std::endl;
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

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    auto gps_time =  gps.getGPSData().time;
    int gps_hour = gps_time.hour;
    int gps_minute = gps_time.minute;
    int gps_second = gps_time.second;

    std::string folderName;

    //std::ostringstream usb_f
    //usb_folder << "../../../../../media/usb/dataset_images/" << str;
    //folderName = usb_folder.str();


    std::ostringstream gps_timestring;
    gps_timestring << str << ";" << gps_hour << ":" << gps_minute << ":" << gps_second;
    folderName = gps_timestring.str();
    //folderName = str;




    const char *c  = folderName.c_str();
    mkdir(c, 0777);
    std::cout << "directory " << c << " created" << std::endl;

    auto cam = getPanoramicCamera();
    const auto cameraRes = cam->getOutputSize();
    std::cout << "camera initialised" << std::endl;

    // unwrapper for camera
   // auto unwrapper = cam->createUnwrapper(cv::Size(WIDTH,HEIGHT));


    cv::Mat imgOrig;
    //--------------------------------->>>>>>>>>>> START RECORDING <<<<<<<<<<<<----------------------------


    // start camera thread
    cv::Mat tmpImage, unwrappedImage, resized;
    cv::Mat outputImage(cv::Size(WIDTH,HEIGHT), CV_8UC3);
    std::thread readCameraThread(&readCameraThreadFunc, cam.get(), std::ref(tmpImage), std::ref(shouldQuit), std::ref(mex));
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // create and open file to write gps coordinates
    std::ofstream coordinates;

    //write headers  for csv
    std::string csvpath = folderName + "/coordinates.csv";
    coordinates.open (csvpath, std::ofstream::trunc);
    coordinates << "speed" << "," << "steering angle" << "," << "gps quality" << "," << "lat" << "," << "lon" << "," << "altitude"  << "," << "roll" << "," << "pitch" << "," << "yaw"  << ",";
    coordinates << "image name" << "," << "timestamp" <<"\n";
    coordinates.close();


    BoBRobotics::BackgroundExceptionCatcher catcher;
    catcher.trapSignals();

    int i = 0;
    BoBRobotics::Stopwatch sw, sw_timestamp;
    std::array<degree_t, 3> angles;
    // for x timestep
    sw_timestamp.start();
    for (;;) {
        // poll from camera thread



        cv::Mat originalImage;
        mex.lock();
        tmpImage.copyTo(originalImage);
        mex.unlock();

        std::ostringstream fileString, folderString;
        fileString << folderName << "/image" << i << ".jpg";
        std::string fileName = fileString.str();
        std::thread imwriteThread{ &saveThread, std::move(originalImage), std::move(fileName) };
        imwriteThread.detach();

        try {
            catcher.check();
            sw.start();
            // get imu data
            angles = imu.getEulerAngles();
            float yaw = angles[0].value();
            float pitch = angles[1].value();
            float roll = angles[2].value();

            // get gps data
            BoBRobotics::GPS::GPSData data = gps.getGPSData();
            BoBRobotics::MapCoordinate::GPSCoordinate coord = data.coordinate;
            BoBRobotics::GPS::TimeStamp time = data.time;
            int gpsQual = (int) data.gpsQuality; // gps quality

            // output results
    	   // if (i%5 == 0) { // only output once ever sec
            std::cout << std::setprecision(10) << "GPS quality: " << gpsQual << " latitude: " << coord.lat.value()<< " longitude: " <<  coord.lon.value()  << " num sats: " << data.numberOfSatellites << " time: " << static_cast<units::time::millisecond_t>(sw_timestamp.elapsed()).value()/1000 << std::endl;


	        folderString << folderName << "/coordinates.csv";


            bot.updateState();
            auto bot_speed = bot.getSpeed();
            auto turn_ang = bot.getTurningAngle();

            //  headers:   speed|steering_angle|gpsQuality|latitude|longitude|altitude|roll|pitch|yaw|imagename|timestamp
            coordinates.open (folderString.str(), std::ofstream::app); // open coordinates.csv file and append
            coordinates << std::setprecision(10) << std::fixed << bot_speed << "," <<  turn_ang.to<double>() << "," << gpsQual << "," << coord.lat.value() << "," << coord.lon.value() << "," << data.altitude.value()  << "," << roll << "," << pitch << "," << yaw  << ",";
            coordinates << "image" << i << ".jpg" << "," << static_cast<units::time::millisecond_t>(sw_timestamp.elapsed()).value() <<"\n";
            coordinates.close();







            int count = (std::chrono::milliseconds(UPDATE_INTERVAL) - sw.elapsed()).count();
            //std::cout << " count " << count/1000000 << " " << std::endl;
            count /= 1000000;
            std::this_thread::sleep_for(std::chrono::milliseconds(count));




            if ( (static_cast<units::time::millisecond_t>(sw_timestamp.elapsed()).value()/1000)>seconds_to_run ) {


                shouldQuit = true;
                readCameraThread.join();

                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_INTERVAL) - sw.elapsed());
        }
        catch(BoBRobotics::GPS::GPSError &e) {
            std::cout << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_INTERVAL) - sw.elapsed());
        }

        i++;
    }



    return 0;

}

