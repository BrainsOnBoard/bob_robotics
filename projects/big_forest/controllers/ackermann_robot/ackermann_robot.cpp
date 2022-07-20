#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Supervisor.hpp>
#include <webots/vehicle/Car.hpp>
#include <webots/vehicle/Driver.hpp>
#include <webots/Device.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include "common/path.h"
#include "common/stopwatch.h"
#include "common/pose.h"
#include "common/bn055_imu.h"
#include "common/background_exception_catcher.h"
#include "navigation/image_database.h"
#include "imgproc/dct_hash.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>


// Standard C includes
#include <ctime>
#include <chrono>
#include <string.h>

#include <plog/Log.h>
#include <plog/Init.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Appenders/DebugOutputAppender.h>
#include "plog/Initializers/RollingFileInitializer.h"

#include "hash_matcher.h"

cv::Mat get_cam_image(const unsigned char *image, int width, int height) {
    /* Matrix which contains the BGRA image from Webots' camera */
    cv::Mat img = cv::Mat(cv::Size(width, height), CV_8UC4);
    img.data = (uchar *)image;
    return img;
}

// time in [ms] of a simulation step
#define TIME_STEP 50 // frame rate 20 fps

#define MAX_SPEED 6.28

#define RESIZED_WIDTH 360
#define RESIZED_HEIGHT 90

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std::literals;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::time;
// entry point of the controller
int bobMain(int argc, char **argv) {


    bool show_images = true;    // show visual
    int seq_length = 70;        // sequence length
    int roll_step = 180;         // number of rotations for a view
    cv::Size unwrapRes(180,45); // resolution of the unwrrapped video
    bool createVideo = false;    // if true, it saves unwrapped video
    bool unwrap = false;         // if true, videos will be unwrapped
    int skipstep = 4;           // skip frames in training matrix
    int testRoute_skipstep =1;
    int num_datasets = 2;
    int testRouteNum = 0;
    int dataset_num = 0;

    double const PI = 3.14159265358979323;

    Route route_vector;
    DTHW sequence_matcher;

    std::unique_ptr<BoBRobotics::Navigation::ImageDatabase::RouteRecorder> recorder;


    Route route = Route(dataset_num,roll_step, skipstep, unwrap, createVideo, unwrapRes);
    std::cout << " creating Hash matrix" << std::endl;
    HashMatrix hashMat(route.nodes, roll_step);
    route.set_hash_matrix(hashMat);
    route_vector = route;
    sequence_matcher = DTHW(route_vector.getHashMatrix(), roll_step, 1000); // init sequence matcher with training matrices
    std::cout << " route is initialized" << std::endl;


    BoBRobotics::BackgroundExceptionCatcher catcher;
    catcher.trapSignals();
    Supervisor *robot = new Driver();
    Driver *driver = driver->getDriverInstance();
    webots::GPS *gps = driver->getGPS("GPS");
    webots::InertialUnit *imu = driver->getInertialUnit("IMU");

    Camera *cm;
    Display *disp = driver->getDisplay("display");
    disp->setColor(0xFFFFFF);
    Keyboard *kb = new Keyboard();

    cm=driver->getCamera("CAM");
    cm->enable(TIME_STEP);
    kb->enable(TIME_STEP);
    gps->enable(TIME_STEP);
    imu->enable(TIME_STEP);
    int width = cm->getWidth();
    int height = cm->getHeight();

    std::cout << "controller started "  << std::endl;

    double speed = 40;
    double key_1 = 49;
    double key_9 = 59;
    int key_R = 82;
    int key_S = 83;
    int key_T = 84;
    bool recording_mode = false;
    bool test_mode = false;
    int last_keypress = -1;
    bool isTestRunning = false;
    bool isRecordRunning = false;
    bool didRecordStop = true;
    bool didTestStop = true;

    tm currentTime;
    const auto rawTime = std::time(nullptr);
    const auto &systemTime = *gmtime(&rawTime);
    currentTime.tm_mday = systemTime.tm_mday;
    currentTime.tm_mon = systemTime.tm_mon;
    currentTime.tm_year = systemTime.tm_year;

    BoBRobotics::Stopwatch sw;
    // feedback loop: step simulation until an exit event is received
    int current_step = 0;
    while (robot->step(TIME_STEP) != -1) {

        cv::Mat current_image = get_cam_image(cm->getImage(),width, height);
        cv::Mat gray_image, resized;
        cv::Mat processed_image;
        const double speed_value = gps->getSpeed();
        const double *pos = gps->getValues();
        const double *orientation = imu->getRollPitchYaw();
        auto yaw = radian_t(orientation[0]);
        auto pitch = radian_t(orientation[1]);
        auto roll = radian_t(orientation[2]);
        auto steering_angle = radian_t(driver->getSteeringAngle());
        // get positions
        double x_pos = pos[0];
        double y_pos = pos[1];
        double z_pos = pos[2];



        if (!current_image.empty()) {

            cv::cvtColor(current_image, gray_image,cv::COLOR_BGRA2GRAY);
            cv::resize(gray_image, resized, cv::Size(RESIZED_WIDTH ,RESIZED_HEIGHT));

        }

        int key = kb->getKey(); // first keypress
        // add velocity
        if (key == 315) {
            driver->setCruisingSpeed(speed);
        }
        // setting target speed with the numbers
        if (key >= key_1 && key < key_9) {
            // the speed is 1-9(keys) * 10
            speed = (double)(key-key_1) * (key_9-key_1);
            driver->setCruisingSpeed(speed);
            std::cout << " speed is " << speed << " km/h" << std::endl;
        }
        // q button press
        if (key == 81) {
            std::cout << "quitting, cleaning up..." << std::endl;

            break;
        }

        // recording mode
        if (key == key_R) {

            if (isTestRunning) {
                std::cout << " you are currently testing, stop it first" << std::endl;
            }
            else if (recording_mode == false && isRecordRunning == false){
                std::cout << "recording mode enabled" << std::endl;
                recording_mode = true;
                isRecordRunning = true;
                BoBRobotics::Navigation::ImageDatabase database{currentTime};
                recorder = database.createRouteRecorder("png", std::vector<std::string>{"Speed",
                                                            "Steering angle [degrees]",
                                                            "Timestamp [ms]"});
                sw.start();
            }


        }

        // stop recording
        if (key == key_S) {

            if (recording_mode && isRecordRunning == true) {
                std::cout << "stop recording" << std::endl;
                recording_mode = false;
                isRecordRunning = false;

            }
            if (test_mode && isTestRunning) {
                std::cout << "stop testing" << std::endl;
                test_mode = false;
                isTestRunning = false;
            }
        }


        // test mode
        if (key == key_T) {
            if (isRecordRunning) {
                std::cout << " you are currently recording, stop it first" << std::endl;
            }
            else if(test_mode == false && isTestRunning == false) {
                std::cout << "test mode" << std::endl;
                test_mode = true;
                isTestRunning = true;

                disp->setColor(0xFF00FF);
                for (auto n : route.nodes) {
                    int xx = meter_t(millimeter_t(n.x)).value();
                    int yy = meter_t(millimeter_t(n.y)).value();
                    float angle_rot1 = 270.0f; // degrees, not radians
                    float radians1 = angle_rot1 * (PI / 180.0f); 	// convert degrees to radians
                    int nx1 = xx * cos(radians1) - yy * sin(radians1);
                    int ny1 = xx * sin(radians1) + yy * cos(radians1);

                    disp->drawPixel(int(ny1)+250, int(nx1)+350);
                }
            }
        }


        if (key == -1) {
            driver->setCruisingSpeed(0.0);
        }

        if (last_keypress != key) {
            if (key != 314 && key != 315 && key != 316) {// exclude arrows from printing
                std::cout << " keypress: " << key << std::endl;
            }
        }
        last_keypress = key;


        int c = kb->getKey(); // second keypress (a key combination's second key)
        if(c>=0) {
            if (c == 314 || key == 314) {
                // turn left
                driver->setSteeringAngle(-0.52);
            }

            else if(c == 316 || key == 316) {
                driver->setSteeringAngle(0.52);
            }
        } else {
            driver->setSteeringAngle(0.0);
        }

        if (recording_mode && isRecordRunning) {
            units::time::millisecond_t elapsed = sw.elapsed();
            std::array<degree_t, 3> attitude{ degree_t(roll), degree_t(pitch), degree_t(yaw) };

            recorder->record({ BoBRobotics::Vector3<meter_t>(meter_t(x_pos),meter_t(y_pos),meter_t(z_pos)), attitude },
                                current_image, speed_value,
                               degree_t(steering_angle).value(),
                                elapsed.value());
            catcher.check();
        }


        if (test_mode && isTestRunning) {

            units::time::millisecond_t elapsed;
            cv::Mat img_gray;
            cv::cvtColor(current_image, img_gray, cv::COLOR_BGRA2GRAY);
            cv::Mat resized_gray;
            cv::resize(img_gray, resized_gray, unwrapRes);

            cv::Mat conv;
            resized.convertTo(conv, CV_32F, 1.0 / 255);
            auto hash = DCTHash::computeHash(conv); // current hash of test set

            std::vector<cv::Mat> img_rots(roll_step); // need to make it with fix size
            std::vector<std::bitset<64>> hash_rots = HashMatrix::getHashRotations(resized ,roll_step, img_rots);

            int rot_size = hash_rots.size();
            int half_rot = rot_size/ 2;

            sequence_matcher.addToShortSequence(hash,seq_length);
            auto seq_index = sequence_matcher.getBestMatch();

            int pixel_step = int(360.0 / (float)roll_step);
            int seq_match_angle = seq_index.second * pixel_step;

            int seq_angle;
            if (seq_match_angle < 180) {
                seq_angle = seq_match_angle;
            } else {
                seq_angle = seq_match_angle -360;
            }

            int min_value;
            auto single_match = HashMatrix::getSingleMatch(hash, route_vector.getHashMatrix(),min_value,roll_step );
            cv::Mat img_match = route_vector.nodes[seq_index.first].image;
            driver->setCruisingSpeed(speed);

            auto rad_angle= radian_t(degree_t(seq_angle));
            driver->setSteeringAngle(-rad_angle.value());


            std::cout << " single = " << single_match.second  << " sequence = " <<seq_index.first <<  "seq_angle= " << seq_match_angle << std::endl;
            cv::imshow("match", img_match);



            //cv::imshow("Gray", resized);

              // can change ordering here << ang matrix >>
            std::vector<std::vector<int>> ang_distances;
            cv::waitKey(1);
/*

            for (int rh = half_rot; rh < rot_size; rh++) {
                std::vector<int> row_distances;
                for (RouteNode route_node : route_vector.nodes) {
                    auto img_hash = route_node.image_hash;
                    auto dist = DCTHash::distance(hash_rots[rh], img_hash);
                    row_distances.push_back(dist);
                }
                ang_distances.push_back(row_distances);
            }
            for (int rh = 0; rh < half_rot; rh++) {
                std::vector<int> row_distances;
                for (RouteNode route_node : route_vector.nodes) {
                    auto img_hash = route_node.image_hash;
                    auto dist = DCTHash::distance(hash_rots[rh], img_hash);
                    row_distances.push_back(dist);
                }
                ang_distances.push_back(row_distances);
            }
*/

          //  cv::Mat prev_costMat = show_angle_matrix(ang_distances);
          //  cv::imshow("angle_dist_mat", prev_costMat);



        }

        disp->setColor(0xFFFFFF);

        //draw trajectory
        int x = pos[0];
        int y = pos[1];
        float angle_rot = 270.0f; // degrees, not radians
        float radians = angle_rot * (PI / 180.0f); 	// convert degrees to radians
        int nx = x * cos(radians) - y * sin(radians);
        int ny = x * sin(radians) + y * cos(radians);
        disp->drawPixel(int(ny)+250, int(nx)+350);




        current_step++;
    }
    //::sync();
    std::cout << "done" << std::endl;
    //delete driver;
    //delete robot;
    //delete gps;
    //delete imu;
    //delete cm;
    //delete disp;
    //delete kb;
    return 0; //EXIT_SUCCESS
}

