#pragma once
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

#include <thrust/device_ptr.h>
#include <thrust/extrema.h>

#include "route_setup.h"

#include "gpu_hasher.h"
#include "gpu_dct.h"


//#include "hash_matcher.cuh"

//#include "dataset_evaluator.cpp"

//   profile with :  nsys profile -w true -t cuda -s none -o nsight_report -f true -x true ./ackermann_robot
cv::Mat get_cam_image(const unsigned char *image, int width, int height) {
    /* Matrix which contains the BGRA image from Webots' camera */
    cv::Mat img = cv::Mat(cv::Size(width, height), CV_8UC4);
    img.data = (uchar *)image;
    return img;
}

// time in [ms] of a simulation step
#define TIME_STEP 32 // frame rate 30 fps


#define MAX_SPEED 6.28

#define RESIZED_WIDTH 256// 255
#define RESIZED_HEIGHT 256 // 64

#define PM_RESIZE_FACTOR 1

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std::literals;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::time;
// entry point of the controller
int main(int argc, char **argv) {



    bool show_images = true;    // show visual

    int roll_step = RESIZED_WIDTH;         // number of rotations for a view
    cv::Size unwrapRes(RESIZED_WIDTH,RESIZED_HEIGHT); // resolution of the unwrrapped video
    const int IMG_WIDTH = unwrapRes.width;
    bool createVideo = false;    // if true, it saves unwrapped video
    bool unwrap = false;         // if true, videos will be unwrapped
   // int skipstep = 11;           // skip frames in training matrix
    int skipstep = 5;
    int seq_length = 128;///2;///skipstep;        // sequence length
    int num_datasets = 2;
    int testRouteNum = 1; // 0 1 2 3
    int dataset_num = 9; // dataset to test
    double const PI = 3.14159265358979323;
    std::string dataset_name= "big_forest/route_zigzag1";

    Route route_vector;



    if (argc >= 2)
    {
        std::istringstream iss( argv[1] );
        int val;

        if (iss >> val)
        {
            // Conversion successful
            skipstep = val;
        }
    }
    if (argc >= 3) {
        std::istringstream iss( argv[2] );
        std::string val;

        if (iss >> val)
        {
            // Conversion successful
            dataset_name = val;
        }
    }



    std::unique_ptr<BoBRobotics::Navigation::ImageDatabase::RouteRecorder> recorder;
    std::unique_ptr<BoBRobotics::Navigation::ImageDatabase> database;

    //Route route;

   // Route route = Route(dataset_num,roll_step, skipstep, unwrap, createVideo, unwrapRes);
    Route route = Route(dataset_name,roll_step, skipstep, unwrap, createVideo, unwrapRes, false, false, {});
    std::cout << " creating Hash matrix" << std::endl;
    std::vector<cv::Mat> training_images;
    for (int i = 0; i < route.nodes.size(); i++) {
        cv::Mat im_col = route.nodes[i].image;
        cv::Mat img_gray;
        cv::cvtColor(im_col, img_gray, cv::COLOR_BGRA2GRAY);
        training_images.push_back(img_gray);
    }
    //unsigned long long int *d_hash_matrix;
    int hash_mat_size =  training_images.size() * roll_step;
    int d_sequence_size = seq_length;

    // init gpu
    GPUHasher g_hasher;
    g_hasher.initGPU(training_images, d_sequence_size, roll_step, RESIZED_WIDTH, RESIZED_HEIGHT);


    BoBRobotics::BackgroundExceptionCatcher catcher;
    catcher.trapSignals();
    Supervisor *robot = new Driver();
    Driver *driver = driver->getDriverInstance();
    webots::GPS *gps = driver->getGPS("GPS");
    webots::InertialUnit *imu = driver->getInertialUnit("IMU");

    Node *rc_car_node = robot->getFromDef("RC_CAR");
    Field *translationField = rc_car_node->getField("translation");

    Camera *cm;
    Display *disp = driver->getDisplay("display");
    Display *disp_dist = driver->getDisplay("DIST_MAT");
    Display *disp_acc = driver->getDisplay("ACCUM_MAT");
    disp->setColor(0x000000);
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
   // while (robot->step(TIME_STEP) != -1) {
    do {
      // begin simulation step computation: send command values to Webots for update
      // leave the loop when the simulation is over
      if (robot->stepBegin(TIME_STEP) == -1)
        break;

        cv::Mat current_image = get_cam_image(cm->getImage(),width, height);
       // cv::GaussianBlur(current_image, current_image, cv::Size(5, 5), 0);
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
            //cv::equalizeHist(gray_image,gray_image);
            cv::resize(gray_image, resized, cv::Size(RESIZED_WIDTH ,RESIZED_HEIGHT),cv::INTER_CUBIC);
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
                database = std::unique_ptr<BoBRobotics::Navigation::ImageDatabase>(new BoBRobotics::Navigation::ImageDatabase{currentTime});

                recorder = database->createRouteRecorder("png", std::vector<std::string>{"Speed",
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
                recorder->save();


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

                disp->setColor(0x00FF00);
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


        // test mode
        if (test_mode && isTestRunning) {

            units::time::millisecond_t elapsed;
            cv::Mat img_gray;
            cv::cvtColor(current_image, img_gray, cv::COLOR_BGRA2GRAY);
            cv::Mat resized_gray;
           // cv::GaussianBlur(img_gray, img_gray, cv::Size(3, 3), 0);
            cv::resize(img_gray, resized_gray, unwrapRes, cv::INTER_CUBIC);

            cv::Mat conv;
            resized.convertTo(conv, CV_32F, 1.0 / 255);


            // ---- gpu sequence ----
            unsigned long long int curr_hash_ptr = g_hasher.addToSequence(conv);
            g_hasher.getDistanceMatrix();
            cv::Mat host_mat1 = g_hasher.downloadDistanceMatrix();
            cv::normalize(host_mat1, host_mat1, 0, 255, cv::NORM_MINMAX);
            host_mat1.convertTo(host_mat1,CV_8UC1);

            // calculate accumulated cost matrix and get best match
            g_hasher.calculate_accumulated_cost_matrix();
            std::pair<int,int> seq_index = g_hasher.getMinIndex(curr_hash_ptr);
            cv::Mat host_mat2 = g_hasher.downloadAccumulatedCostMatrix();
            cv::normalize(host_mat2, host_mat2, 0, 255, cv::NORM_MINMAX);
            host_mat2.convertTo(host_mat2,CV_8UC1);
            cv::Mat combined, comb4, comb4_acc;
            cv::cvtColor(host_mat1, comb4, cv::COLOR_GRAY2BGRA );
            cv::cvtColor(host_mat2, comb4_acc, cv::COLOR_GRAY2BGRA );


            int dw = disp_dist->getWidth();
            int dh = disp_dist->getHeight();
            int aw = disp_acc->getWidth();
            int ah =disp_acc->getHeight();
            cv::resize(comb4, comb4, {dw,dh});
            cv::resize(comb4_acc, comb4_acc, {aw,ah});

            ImageRef *ir = disp_dist->imageNew(dw, dh, comb4.data, Display::BGRA);
            ImageRef *ir_acc = disp_acc->imageNew(aw, ah, comb4_acc.data, Display::BGRA);
            disp_dist->imagePaste(ir, 0, 0, false);
            disp_acc->imagePaste(ir_acc, 0, 0, false);
            disp_dist->imageDelete(ir);
            disp_acc->imageDelete(ir_acc);


            //////--------------------------------------------------------

            int pixel_step = int(IMG_WIDTH / (float)roll_step);
            int seq_match_angle = seq_index.second * pixel_step;

            int seq_angle;
            if (seq_match_angle <= 180) {
                seq_angle = seq_match_angle;
            } else {
                seq_angle = seq_match_angle -360;
            }

            if (seq_angle > 30) {
                seq_angle = 30;
            }

            if (seq_angle <= -30) {
                seq_angle = -30;
            }



            int min_value;
        //  auto single_match = HashMatrix::getSingleMatch(hash, route_vector.getHashMatrix(),min_value,roll_step );
            //cv::Mat img_match = route_vector.nodes[seq_index.first].image;
            if (seq_angle <= 0) {
                speed = 45+seq_angle;
            }
            else {
                speed = 45-seq_angle;
            }
            driver->setCruisingSpeed(speed);

            auto rad_angle= radian_t(degree_t(seq_angle));
            driver->setSteeringAngle(rad_angle.value());

            std::cout << " sequence = " <<seq_index.first <<  "seq_angle= " << seq_angle << std::endl;
            //  cv::imshow("match", img_match);
            // can change ordering here << ang matrix >>
            std::vector<std::vector<int>> ang_distances;
            cv::waitKey(1);
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
 //   }
    } while (robot->stepEnd() != -1);

    std::cout << "done" << std::endl;

    return 0; //EXIT_SUCCESS
}

