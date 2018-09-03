#include <vector>
#include <list>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <chrono>
#include <csignal>
#include <thread>
#include <algorithm>


// BoB robotics includes
#include "../vicon/capture_control.h"
#include "../vicon/udp.h"
#include "../robots/norbot.h"
#include "../video/see3cam_cu40.h"
#include "../imgproc/opencv_unwrap_360.h"

using namespace BoBRobotics::Vicon;
using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Video;
using namespace std::literals;

#define PI 3.14159265359

class robotPositioner {

private:

    // tracking and camera variables
    std::list<std::vector<double>> m_coordinates;             // goal coordinates
    UDPClient<ObjectData> *m_vicon;                           // vicon 
    CaptureControl *m_viconCaptureControl;                    // vicon capture
    BoBRobotics::Robots::Norbot *m_bot;                       // robot interface
    See3CAM_CU40 *m_cam;                                      // camera
    OpenCVUnwrap360 m_unwrapper;                              // image unwrapper
    int m_imageNumber;                                        // image id number 
    bool m_hasCamera;                                         // a boolean to indicate whether we have camera
  
    // Robot variables
    double m_pos_X;                                           // robot's x position
    double m_pos_Y;                                           // robot's y position
    double m_pos_Z;                                           // robot's z position
    double m_heading;                                         // heading (angle) of the robot
    double m_goalPositionX;                                   // goal position x
    double m_goalPositionY;                                   // goal position y
    double m_goalPositionZ;                                   // goal position z
    double m_bearingFromGoal;                                 // bearing (angle) from goal coordinate
    double m_distanceFromGoal;                                // Euclidean distance from goal coordinate
    double m_goalAngle;                                       // angle to turn after finding the correct location
    double m_bearingFinal;                                    // heading converted to range <-180, 180> from <0, 360>

    // free variables for the user to set
    unsigned int m_vicon_udp_client_port;                     // udp streaming port number
    std::string m_vicon_capture_control_ip_address;           // vicon capture control ip address
    std::string m_vicon_capture_control_executable_path;      // vicon capture control program executable path
    unsigned int m_capture_control_port;                      // vicon capture control port number
    double m_robot_r;                                         // robot's wheel radius
    double m_robot_D;                                         // robots' axis length
    double m_threshold_distance;                              // threshold distance for the robot to start slowing down
    double m_stopping_distance;                               // if the robot's distance from goal < stopping dist, robot stops
    std::string m_video_device;                               // video device path (usually /dev/device0 )
    double m_k1;                                              // curveness of the path to the goal
    double m_k2;                                              // speed of turning on the curves
    double m_alpha;                                           // causes more sharply peaked curves 
    double m_beta;                                            // causes to drop velocity if 'k'(curveness) increases
    double m_max_velocity;                                    // max velocity

    /**
    * sets up the vicon system so it listens to the broadcast 
    */
    void setupVicon() {

        //vicon = new UDPClient<ObjectData>(51001);
        //viconCaptureControl = new CaptureControl( "192.168.1.100", 3003,"c:\\users\\ad374\\Desktop");

        // setup vicon and capture control
        m_vicon = new UDPClient<ObjectData>(m_vicon_udp_client_port);
        m_viconCaptureControl = new CaptureControl( 
            m_vicon_capture_control_ip_address,
            m_capture_control_port,
            m_vicon_capture_control_executable_path
        );

        // keep trying to get stream of data
        while(m_vicon->getNumObjects() == 0) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::cout << "Waiting for object" << std::endl;
        }

        // start recording 
        if(!m_viconCaptureControl->startRecording("test1")) {
            std::cout << "failed to start recording" << std::endl;
        } else {
            std::cout << "vicon finished setting up" << std::endl;
        }
    }


    // sets up the 360 SeeCam
    void setupCamera360() {
        //const std::string device = "/dev/video0";
        const std::string device = m_video_device;

        m_cam = new See3CAM_CU40(device, See3CAM_CU40::Resolution::_1280x720);
        const cv::Size unwrapSize(180,50); // calculate camera input dimensions

        const cv::Mat bubblescopeMask = See3CAM_CU40::createBubblescopeMask(m_cam->getSuperPixelSize());
        m_cam->autoExposure(bubblescopeMask);
        m_unwrapper = m_cam->createUnwrapper(unwrapSize);
    }

    // gets vicon data
    double getViconData() {

        auto objectData = m_vicon->getObjectData(0);
   
        BoBRobotics::Vector3<millimeter_t> translation = objectData.getPosition();
        BoBRobotics::Vector3<radian_t> rotation = objectData.getAttitude();

        m_pos_X = translation[0].value();
        m_pos_Y = translation[1].value();
        m_heading = rotation[0].value() * 180.0/PI;
    }


    // updates the range and bearing from the goal location
    void updateRangeAndBearing() {

        getViconData(); // update variables

        double delta_x = m_goalPositionX - m_pos_X;
        double delta_y = m_goalPositionY - m_pos_Y;
        std::vector<double> delta;
        delta.push_back(delta_x);
        delta.push_back(delta_y);

        // calculating the distance form the goal
        double q = std::inner_product(std::begin(delta), std::end(delta), std::begin(delta), 0.0);
        m_distanceFromGoal = sqrt(q);
        m_bearingFromGoal = atan2(delta_y,delta_x)*180.0/PI - m_heading - 90.0;
        
        // changing from <0,360> to <-180, 180>
        m_bearingFromGoal = angleWrapAround(m_bearingFromGoal);

    } 

    template <class T>
    T angleWrapAround(T angle) {
        // changing from <0,360> to <-180, 180>
        if (angle <= 0) { angle += 360; } 
        if (angle > 180) { angle -= 360; }
        return angle;
    }

    template <class T, class S>
    bool closeTo(const T num, const S threshold) {
        if (num > -threshold && num < threshold) {
            return true;
        } else {
            return false;
        }
    }

    /**
    * reads in the coordinates from a CSV file and puts them in a list
    */
    void readCoordinatesFromFile(const std::string csv_file) {
        std::ifstream in(csv_file);
        std::vector<std::vector<double>> fields;
        if (in) {
            std::string line;
            while (getline(in, line)) {
                std::stringstream sep(line);
                std::string field;
                fields.push_back(std::vector<double>());
                while (getline(sep, field, ',')) {
                    fields.back().push_back(stof(field));
                }
            }
        }

        // saving coordinates from csv
        for (auto row : fields) {	    	
            std::vector<double> temp_vec;
            for (double field : row) {
                temp_vec.push_back(field);         	
            }
            m_coordinates.push_back(temp_vec);
        }
    }

    // drives the robot the desired pose (x,y,angle)
    void driveRobotToGoalLocation() {
        bool stopFlag = true;

        while (stopFlag) {
            // updates the range and bearing
            updateRangeAndBearing();

            // orientation of Target with respect to the line of sight from the observer to the target
            double theta = m_heading + m_bearingFromGoal - m_goalAngle; 
            theta = angleWrapAround(theta);

            // curveness value: k
            double k = -(1/m_distanceFromGoal)* ( m_k2* (m_bearingFromGoal- atan(-m_k1*theta*PI/180.0)*180.0/PI) + 
                1+(m_k1/(1+pow(m_k1*theta,2)))*sin(m_bearingFromGoal*PI/180.0)*180.0/PI);

            // current velocity
            double v = m_max_velocity/(1+m_beta*pow(abs(k),m_alpha));

            // formula for curvy path
            double w = k*v;
        
            // if we are very close, don't do any large turns
            if (m_distanceFromGoal < m_threshold_distance && closeTo(m_bearingFromGoal, 5) ) { w = 0.001; }
            
            // if we are about 3cm close, stop the robot
            if (m_distanceFromGoal < m_stopping_distance && closeTo(m_bearingFromGoal,4)) {
                stopFlag = false;
                m_bot->tank(0,0);
            } else {
                driveRobotWithVelocities(v,w, m_robot_r, m_robot_D);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // drives the robot wheels using v and w as velocities which get translated to 
    // indiviual wheel velocities. Conversion from these velocities to individual 
    // wheel velocities is done via solving simultaneous equations
    // parameters:
    //             v      : translational velocity
    //             w      : rotational velocity
    //             robot_r: wheel radius
    //             robot_d: axis length
    template <class T, class S>
    void driveRobotWithVelocities(const T v, const T w, const S robot_r, const S robot_d) {
        // v = r * (Vl+Vr)/2
        // w = r * (Vr-Vl)/D
        T a =  robot_r/2;
        T b =  robot_r/2;
        T c =  robot_r / robot_d;
        T d = -(robot_r / robot_d);

        T e=v;
        T f=w;
        T Vl,Vr;

        // determinant
        T det = a*d - b*c;

        // if the determinant is not zero, there is a solution
        if (det != 0) {
            Vl = (e*d - b*f)/det;
            Vr = (a*f - e*c)/det;
        }
        // drive robot 
        m_bot->tank(Vl, Vr);
    }

    // captures a frame
    void captureCameraFrame() {
        const cv::Size unwrapSize(180,50);
        cv::Mat output(m_cam->getSuperPixelSize(), CV_8UC1);
        cv::Mat unwrapped(unwrapSize, CV_8UC1);

        if (m_cam->captureSuperPixelGreyscale(output)) {
            m_unwrapper.unwrap(output, unwrapped);
            char fileName[255];
            sprintf(fileName, "image_%u.png", m_imageNumber++);
            cv::imwrite(fileName, unwrapped);
            std::cout << " image " << m_imageNumber << " captured " << std::endl;
        }
    }

//-----------------PUBLIC API---------------------------------------------------------------------
public:

    // constructor
    robotPositioner(
        unsigned int vicon_udp_client_port,
        std::string  vicon_capture_control_ip_address,
        std::string  vicon_capture_control_executable_path,
        unsigned int capture_control_port,
        double       robot_r,
        double       robot_D,
        double       threshold_distance,
        double       stopping_distance,
        std::string  video_device,
        double       k1,
        double       k2,
        double       alpha,
        double       beta,
        double       max_velocity
        ) : m_vicon_udp_client_port(vicon_udp_client_port),
            m_vicon_capture_control_ip_address(vicon_capture_control_ip_address),
            m_vicon_capture_control_executable_path(vicon_capture_control_executable_path),
            m_capture_control_port(capture_control_port),
            m_robot_r(robot_r),
            m_robot_D(robot_D),
            m_threshold_distance(threshold_distance),
            m_stopping_distance(stopping_distance),
            m_video_device(video_device),
            m_k1(k1),
            m_k2(k2),
            m_alpha(alpha),
            m_beta(beta),
            m_max_velocity(max_velocity)
        {

        m_imageNumber = 0;
        m_bot = new BoBRobotics::Robots::Norbot(); // init robot
        setupVicon();                              // init vicon system

        // setup 360 camera if exists
        try {
            setupCamera360();
            std::cout << "camera 360 finished setting up" << std::endl;
        } catch(...) {
            std::cout << "camera cannot be initialised, maybe it does not exists..." << std::endl;
            m_hasCamera = false;
        }
    }

    //destructor
    ~robotPositioner() {

        if(!m_viconCaptureControl->stopRecording("test1")) {
            std::cout << "failed to stop recording" << std::endl;
        }

        delete m_vicon;
        delete m_viconCaptureControl;
        delete m_bot;

        if (m_hasCamera) {
            delete m_cam;
        }
    }

    /**
    *	starts the robot positioning program. The function expects a csv file with 4 columns |x|y|z|a|
    *   where 'xyz' are the coordinates of the goal location and 'a' is the final angle.
    *   the robot will visit all the coordinates in the file, then snap an image with the camera at each 
    *   location if the camera is attached.
    **/
    void startSession(const std::string fileName) {

        // read in the coordinates from file to a list of vectors
        readCoordinatesFromFile(fileName);
        
        // for all the coordinates in the csv file
        int locationNumber = 1;
        for (std::vector<double> element : m_coordinates) {

            std::cout << " visiting location " << locationNumber << std::endl;
            locationNumber++;

            m_goalPositionX = element[0];
            m_goalPositionY = element[1];
            m_goalAngle = element[3];

            // go to location 
            driveRobotToGoalLocation();

            // take a picture
            if (m_hasCamera) {
                captureCameraFrame();
                std::cout << " image " << m_imageNumber << " snapped " << std::endl;
            }

            std::cout << " arrived to location [x = " << m_goalPositionX << ", y = " << m_goalPositionY << "]"
                << " distance error =" << m_distanceFromGoal << " heading error =" << abs(m_heading-m_goalAngle) << std::endl;

            // stop robot 
            m_bot->tank(0,0);

            // wait 1 sec
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        }
    }
};
