// standard includes
#include<string>
#include<vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

// BoB includes
#include "common/main.h"
#include "common/stopwatch.h"
#include "common/background_exception_catcher.h"
#include "plog/Log.h"
#include "common/macros.h"
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
#include <sstream>

using namespace BoBRobotics;
using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Video;
using namespace std::literals;


void readCameraThreadFunc(BoBRobotics::Video::Input *cam,
                        cv::Mat &img,
                        std::atomic<bool> &shouldQuit,
                        std::mutex &mutex)
{

    while(!shouldQuit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::lock_guard<std::mutex> lock(mutex);
        cam->readFrame(img);
    }
}

int bobMain(int argc, char* argv[])
{
    constexpr auto UPDATE_INTERVAL = 100ms;
    using degree_t = units::angle::degree_t;

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
        LOGD << "running for " << seconds_to_run << " seconds";
    }

    // check to see if we get valid coordinates by checking GPS quality
    int gps_hour,gps_minute,gps_second;
    BoBRobotics::GPS::TimeStamp gps_time;
    bool found = false;
    while (numTrials > 0 && !found) {
        try {
            if (gps.getGPSData().gpsQuality != BoBRobotics::GPS::GPSQuality::INVALID) {

                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                // getting time data to set folder name
                gps_time =  gps.getGPSData().time;
                gps_hour = gps_time.hour;
                gps_minute = gps_time.minute;
                gps_second = gps_time.second;
                found = true;
                LOGD << " we have a valid measurement";

            } else {
                numTrials--;
            }

        } catch(BoBRobotics::GPS::GPSError &e) {
            LOGW << " measuring failed, trying again in 1 second " << "[" << maxTrials - numTrials << "/" << maxTrials << "]";
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            numTrials--;
        }
    }

    if (numTrials == 0) {
        LOGW << " There is no valid gps measurement, please try waiting for the survey in to finish and restart the program ";
        return EXIT_FAILURE;
    }
    std::stringstream ss;
    ss << "imgdataset_"  << str << "_" << gps_hour << "-" << gps_minute << "-" << gps_second;
    const auto folderName = ss.str();
    const char *c  = folderName.c_str();
    BOB_ASSERT(mkdir(c, 0777) == 0);
    LOGD << "directory " << c << " created";
    auto cam = getPanoramicCamera();
    const auto cameraRes = cam->getOutputSize();
    LOGD << "camera initialised " << cameraRes;



    //--------------------------------->>>>>>>>>>> START RECORDING <<<<<<<<<<<<----------------------------
    // start camera thread
    cv::Mat tmpImage;
    std::thread readCameraThread(&readCameraThreadFunc, cam.get(), std::ref(tmpImage), std::ref(shouldQuit), std::ref(mex));
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    BoBRobotics::BackgroundExceptionCatcher catcher;
    catcher.trapSignals();

    int i = 0;
    BoBRobotics::Stopwatch sw, sw_timestamp;
    std::array<degree_t, 3> angles;
    // for x timestep - starting stopwatch
    sw_timestamp.start();

    std::ofstream coordinates; // file handle
    std::ostringstream folderString;
    folderString << folderName << "/coordinates.csv";
    //  headers:   speed|steering_angle|gpsQuality|latitude|longitude|altitude|roll|pitch|yaw|imagename|timestamp
    //write headers  for csv
    coordinates.open (folderString.str(), std::ofstream::trunc);
    coordinates << "speed" << "," << "steering angle" << "," << "gps quality" << "," << "lat" << "," << "lon" << "," << "altitude"  << "," << "roll" << "," << "pitch" << "," << "yaw"  << ",";
    coordinates << "image name" << "," << "timestamp" <<"\n";

    for (;;) {
        // poll from camera thread
        sw.start();
        cv::Mat originalImage;
        mex.lock();
        tmpImage.copyTo(originalImage);
        mex.unlock();
        std::ostringstream fileString;
        fileString << folderName << "/image" << i << ".jpg";
        std::string fileName = fileString.str();


        if (!cv::imwrite(fileName, originalImage)) {
            LOGW << "Could not save image file";
        }
        // get imu data
        float roll,pitch,yaw;
        try {
            angles = imu.getEulerAngles();
            yaw = angles[0].value();
            pitch = angles[1].value();
            roll = angles[2].value();
        } catch(...) {

            // if we can't read the imu, we just write nan values
            roll = NAN;
            pitch = NAN;
            yaw = NAN;
        }

        // read speed and steering angle from robot
        float bot_speed;
        degree_t turn_ang;
        try {
            bot.updateState();
            bot_speed =  bot.getSpeed();
            turn_ang = bot.getTurningAngle();
        } catch(...) {
            // if we can't read speed or angle, we just write nan values
            bot_speed = NAN;
            turn_ang = 0_deg;
        }
        try {
            catcher.check();

            // get gps data
            BoBRobotics::GPS::GPSData data = gps.getGPSData();
            BoBRobotics::MapCoordinate::GPSCoordinate coord = data.coordinate;
            int gpsQual = (int) data.gpsQuality; // gps quality

            // output results
            LOGI << std::setprecision(10)
                << "GPS quality: " << gpsQual
                << " latitude: " << coord.lat.value()
                << " longitude: " <<  coord.lon.value()
                << " num sats: " << data.numberOfSatellites
                << " time: " << static_cast<units::time::millisecond_t>(sw_timestamp.elapsed()).value()/1000 << std::endl;

            coordinates << std::setprecision(10) << std::fixed
                << bot_speed << ","
                <<  turn_ang.to<double>() << ","
                << gpsQual << ","
                << coord.lat.value() << ","
                << coord.lon.value() << ","
                << data.altitude.value()  << ","
                << roll << ","
                << pitch << ","
                << yaw  << ","
                << "image" << i << ".jpg" << "," << static_cast<units::time::millisecond_t>(sw_timestamp.elapsed()).value() <<"\n";
            // calculating time to wait if loop was done faster thann update time
            std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_INTERVAL) - sw.elapsed());

            if ( (static_cast<units::time::second_t>(sw_timestamp.elapsed()).value())>seconds_to_run ) {
                // if we quit, close file handles, threads etc...
                coordinates.close();
                shouldQuit = true;
                readCameraThread.join();
                break;
            }


        }
        catch(BoBRobotics::GPS::GPSError &e) {
            LOGW << e.what();
            coordinates << std::setprecision(10) << std::fixed
                << bot_speed << ","
                << turn_ang.to<double>() << ","
                << NAN << ","
                << NAN << ","
                << NAN << ","
                << NAN  << ","
                << roll << ","
                << pitch << ","
                << yaw  << ","
                << "image" << i << ".jpg" << "," << static_cast<units::time::millisecond_t>(sw_timestamp.elapsed()).value() <<"\n";
        }

        i++;
    }
    return EXIT_SUCCESS;
}

