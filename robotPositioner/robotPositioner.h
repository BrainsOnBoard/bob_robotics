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
	std::list<std::vector<double>> coordinates;	// goal coordinates
    UDPClient<ObjectData> *vicon;				// vicon 
    CaptureControl *viconCaptureControl; 		// vicon capture
    BoBRobotics::Robots::Norbot *bot; 			// robot interface
    See3CAM_CU40 *cam;				  			// camera
    OpenCVUnwrap360 unwrapper;		  			// image unwrapper
    int imageNumber;				  			// image id number 
    bool hasCamera;								// a boolean to indicate whether we have camera

  
  	// Robot variables
    double pos_X;								// robot's x position
    double pos_Y;								// robot's y position
    double pos_Z;								// robot's z position
    double heading;								// heading (angle) of the robot
    double goalPositionX;						// goal position x
    double goalPositionY;						// goal position y
    double goalPositionZ;						// goal position z
    double bearingFromGoal;						// bearing (angle) from goal coordinate
    double distanceFromGoal;					// Euclidean distance from goal coordinate
    double goalAngle; 							// angle to turn after finding the correct location
    double bearingFinal;						// heading converted to range <-180, 180> from <0, 360>
   

    float k1_in, k2_in, alpha_in, beta_in, max_vel;							// used for user input to set k1 and k2 values


    /**
    * sets up the vicon system so it listens to the broadcast 
    */
	void setupVicon() {

		vicon = new UDPClient<ObjectData>(51001);
		viconCaptureControl = new CaptureControl(
			"192.168.1.100", 3003, 
    		"c:\\users\\ad374\\Desktop"
    		);

		while(vicon->getNumObjects() == 0) {
        	std::this_thread::sleep_for(std::chrono::seconds(1));
       		std::cout << "Waiting for object" << std::endl;
       	}

       	// start recording 
       	if(!viconCaptureControl->startRecording("test1")) {
        	std::cout << "failed to start recording" << std::endl;
		} else {
			std::cout << "vicon finished setting up" << std::endl;
		}

	
	}


	// sets up the 360 SeeCam
	void setupCamera360() {
		const std::string device = "/dev/video0";
		cam = new See3CAM_CU40(device, See3CAM_CU40::Resolution::_1280x720);
		const cv::Size unwrapSize(180,50); // calculate camera input dimensions

		const cv::Mat bubblescopeMask = See3CAM_CU40::createBubblescopeMask(cam->getSuperPixelSize());
		cam->autoExposure(bubblescopeMask);
		unwrapper = cam->createUnwrapper(unwrapSize);
	}

	// gets vicon data
	double getViconData() {

		auto objectData = vicon->getObjectData(0);
   
		BoBRobotics::Vector3<millimeter_t> translation = objectData.getPosition();
		BoBRobotics::Vector3<radian_t> rotation = objectData.getAttitude();

		pos_X = translation[0].value();
		pos_Y = translation[1].value();
		heading = rotation[0].value() * 180/PI;
	}


	// updates the range and bearing from the goal location
	void updateRangeAndBearing() {

		getViconData(); // update variables
	
		double delta_x = goalPositionX - pos_X;
		double delta_y = goalPositionY - pos_Y;
		std::vector<double> delta;
		delta.push_back(delta_x);
		delta.push_back(delta_y);

		// calculating the distance form the goal
		double q = std::inner_product(std::begin(delta), std::end(delta), std::begin(delta), 0.0);
		distanceFromGoal = sqrt(q);
		bearingFromGoal = atan2(delta_y,delta_x)*180/PI - heading - 90;
		
		bearingFromGoal = angleWrapAround(bearingFromGoal);

	} 

	template <class T>
	T angleWrapAround(T angle) {
		// changing from <0,360> to <-180, 180>
		if (angle <= 0) { angle += 360; } 
		if (angle > 180) { angle -= 360; }
		return angle;

	}

	template <class T, class S>
	bool closeTo(T num, S threshold) {
		if (num > -threshold && num < threshold) {
			return true;
		} else {
			return false;
		}
	}

	/**
	* reads in the coordinates from a CSV file and puts them in a list
	*/
	void readCoordinatesFromFile(std::string csv_file) {
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
			coordinates.push_back(temp_vec);
    	}
	}

	// drives the robot the desired pose (x,y,angle)
	void driveRobotToGoalLocation() {
		bool stopFlag = true;

		float w_prev;

		while (stopFlag) {
			updateRangeAndBearing();
			float k1 = k1_in;  // curvness of the path
			float k2 = k2_in;  // speed of turning
			
			// orientation of Target with respect to the line of sight from the observer to the target
			float theta = heading + bearingFromGoal - goalAngle; 
			theta = angleWrapAround(theta);
			//float v = 5.0;

			//float v = 5;

			float k = -(1/distanceFromGoal)* ( k2* (bearingFromGoal- atan(-k1*theta*PI/180)*180/PI) + 
				1+(k1/(1+pow(k1*theta,2)))*sin(bearingFromGoal*PI/180)*180/PI);

			float alpha = alpha_in;
			float beta = beta_in;
			float v = max_vel/(1+beta*pow(abs(k),alpha));

			// formula for curvy path
			float w = k*v;
	
			// if we are very close, don't do any large turns
	        if (distanceFromGoal<300 && closeTo(bearingFromGoal, 5) ) { w = 0.001; v = 4; }
	        //else if (distanceFromGoal<300) { v = distanceFromGoal/30; }

	        
	        // if we are about 3cm close, stop the robot
	        if (distanceFromGoal < 30 && closeTo(bearingFromGoal,4)) {
				stopFlag = false;
				bot->tank(0,0);
			} else {
				driveRobotWithVelocities(v,w);
			}

	        std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}

	// drives the robot wheels using v and w as velocities which get translated to 
	// indiviual wheel velocities
	void driveRobotWithVelocities(float v, float w) {
		// v = r * (Vl+Vr)/2
		// w = r * (Vr-Vl)/D
		// r = 7.5  D = 10.4 

		//ax+by = e   -- > 3.4x + 3.4y = v
		//cx+dy = f   -- > 0.65y - 0.65x = w

		if (v >= 10) {v = 10; }

		float a = 3.4;
		float b = 3.4;
		float c = 0.65;
		float d = -0.65;

		float e=v;
		float f=w;

		float Vl,Vr;

		float det = a*d - b*c;

		if (det != 0) {
			Vl = (e*d - b*f)/det;
			Vr = (a*f - e*c)/det;
		}
		// drive robot 
		bot->tank(Vl, Vr);
	}

	

	// captures a frame
	void captureCameraFrame() {
		const cv::Size unwrapSize(180,50);
		cv::Mat output(cam->getSuperPixelSize(), CV_8UC1);
		cv::Mat unwrapped(unwrapSize, CV_8UC1);

		if (cam->captureSuperPixelGreyscale(output)) {
			unwrapper.unwrap(output, unwrapped);
			char fileName[255];
			sprintf(fileName, "image_%u.png", imageNumber++);
			cv::imwrite(fileName, unwrapped);
			std::cout << " image " << imageNumber << " captured " << std::endl;
		}

	}

	

public:

	// constructor
	robotPositioner() {

		imageNumber = 0;
		bot = new BoBRobotics::Robots::Norbot(); // init robot
		setupVicon();

		try {
			setupCamera360();
			std::cout << "camera 360 finished setting up" << std::endl;
		} catch(...) {
			std::cout << "camera cannot be initialised, maybe it does not exists..." << std::endl;
			hasCamera = false;
		}

		std::cout << " enter k1 for the positioner [How curvy is the curve?]" << std::endl;
		std::cin >> k1_in;
		std::cout << " enter k2 for the positioner [Speed of steering]" << std::endl;
		std::cin >> k2_in;
		std::cout << " enter alpha " << std::endl;
		std::cin >> alpha_in;
		std::cout << " enter beta " << std::endl;
		std::cin >> beta_in;
		std::cout << " max velocity " << std::endl;
		std::cin >> max_vel;
	}

	//destructor
	~robotPositioner() {

		if(!viconCaptureControl->stopRecording("test1")) {
			std::cout << "failed to stop recording" << std::endl;
		}

		delete vicon;
		delete viconCaptureControl;
		delete bot;

		if (hasCamera) {
			delete cam;
		}
	}

	

	/**
	*	starts the robot positioning program. The function expects a csv file with 4 columns |x|y|z|a|
	*   where 'xyz' are the coordinates of the goal location and 'a' is the final angle.
	*   the robot will visit all the coordinates in the file, then snap an image with the camera at each 
	*   location if the camera is attached.
	**/
	void startSession(std::string fileName) {

		// read in the coordinates from file to a list of vectors
		readCoordinatesFromFile(fileName);
		
		// for all the coordinates in the csv file
		int locationNumber = 1;
		for (std::vector<double> element : coordinates) {

			std::cout << " visiting location " << locationNumber << std::endl;
			locationNumber++;

			goalPositionX = element[0];
			goalPositionY = element[1];
			goalAngle = element[3];

			// go to location 
			driveRobotToGoalLocation();

			
			// take a picture
			if (hasCamera) {
				captureCameraFrame();
				std::cout << " image " << imageNumber << " snapped " << std::endl;
			}

			std::cout << " arrived to location [x = " << goalPositionX << ", y = " << goalPositionY << "]"
				<< " distance error =" << distanceFromGoal << " heading error =" << abs(heading-goalAngle) << std::endl;

			// stop robot 
			bot->tank(0,0);

			// wait 1 sec
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		}

	
	}

};