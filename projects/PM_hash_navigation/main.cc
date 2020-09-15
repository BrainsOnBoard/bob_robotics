//main

// BoB robotics includes
#include "common/logging.h"
#include "common/timer.h"
#include "imgproc/opencv_unwrap_360.h"
#include "video/panoramic.h"
#include "pm_hasher.h"
#include "snapshot_db.h"

// Standard C++ includes
#include <iostream>

// Standard C includes
#include <cmath>
#include "pm_control.h"

#include <opencv2/objdetect.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <chrono>
 
using namespace BoBRobotics;
using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Video;

int pixelToAngle(int pixel, int imageWidth) {
    int angle = 0;
    float oneDegreeVal =(float)imageWidth / 360.0;

    angle = (int)( (float)pixel/oneDegreeVal);
    return angle;
}

int angleNormalise(int angle) {
	angle = (angle + 180) % 360;
	if (angle < 0) {
		angle += 360;
	}
	return angle-180;
}




int main(int argc, char **argv)
{
    const int WIDTH = 320;
    const int HEIGHT = 90;
    const cv::Size unwrapRes(190, 32); // 64,16 for panoramic
    //const cv::Size unwrapRes(64,48);
    const unsigned int outputScale = 3;
    // Create panoramic camera and suitable unwrapper
    auto cam = getPanoramicCamera();

    //auto cam = std::make_unique<OpenCVInput>(0, cv::Size(1440, 1440), "pixpro_usb");
    auto unwrapper = cam->createUnwrapper(unwrapRes);
    const auto cameraRes = cam->getOutputSize();
    // Create images
    cv::Mat originalImage(cameraRes, CV_8UC3);
    

    //cv::namedWindow("Unwrapped", cv::WINDOW_NORMAL);
    //cv::resizeWindow("Unwrapped", unwrapRes.width * outputScale,
//                     unwrapRes.height * outputScale);

    
    
   


    PM_Control controller; 
    int mode;
    int isPanoramic = 0;
    if (argc >= 2) {
        mode = atoi(argv[1]);
        if (argc >= 3) {
            isPanoramic = atoi(argv[2]);
        }
        
    }
    Snapshot_DB datab;

    if (mode == 0) {
	std::cout << "Training..." << std::endl;
	cv::namedWindow("Unwrapped", cv::WINDOW_NORMAL);
    	cv::resizeWindow("Unwrapped", unwrapRes.width * outputScale, unwrapRes.height* outputScale);

    }	
    else if (mode == 1) {
        std::cout << "Testing..." << std::endl;
        datab.loadTrainingData();
       // cv::namedWindow("Matched", cv::WINDOW_NORMAL);
       // cv::resizeWindow("Matched", unwrapRes.width * outputScale,
        //            unwrapRes.height * outputScale);

    }

  
    // ----- camera loop ---------------
    unsigned int frame = 0;




    for(frame = 0;; frame++) {
        // Read from camera
        if(!cam->readFrame(originalImage)) {
            return EXIT_FAILURE;
        }
        //cv::blur(originalImage, originalImage, cv::Size(3,3));
        // Unwrap
        cv::Mat outputImage(unwrapRes, CV_8UC3);
        cv::cvtColor(originalImage, originalImage, cv::COLOR_RGB2GRAY);
        // Apply Histogram Equalization
       // cv::equalizeHist( originalImage, originalImage );
        originalImage.convertTo(originalImage, CV_32F, 1.0/255);
        
        if (isPanoramic) {
            unwrapper.unwrap(originalImage, outputImage);
        } else {
            cv::resize(originalImage,outputImage, unwrapRes,0,0);
        }
        
        
       
        
        if (mode == 0) datab.addSnapshot(outputImage);
        else {
           // if (outputImage.width() > 0 && outputImage.height() > 0) {
               
                //Match match =  datab.findBestMatchAngleBruteForce(outputImage); 
                cv::Mat rotatedView;
                cv::Mat resizedRotatedView;
                Match match =  datab.findBestMatchRotation(outputImage, rotatedView);

                // show best rotated view
		std::stringstream strs;
		int anglepix = pixelToAngle(match._best_rotation, unwrapRes.width);
		int ang = pixelToAngle(anglepix, unwrapRes.width);

		std::cout << " ang " << ang << std::endl;

		int angleNormalised = angleNormalise(anglepix);
		strs << " angle " << angleNormalised;  // print normalised angle
		cv::resize(rotatedView,resizedRotatedView, {WIDTH*outputScale,HEIGHT*outputScale});
		cv::putText(
			resizedRotatedView, 
			strs.str(),
		        cv::Point(20,20), // Coordinates
		        cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
		        1.3, // Scale. 2.0 = 2x bigger
		        {255,0,255}, // BGR Color
		        1 // Line Thickness (Optional)
                );
              //  cv::imshow("rotated view", resizedRotatedView);
                

		// move with car 
		controller.updateMotors(angleNormalised);
		




                int m_index = match._match_index;
                std::stringstream ss;
                ss << "training_data/training_image" << m_index << ".jpg";
                std::string path;
                ss >> path;
                cv::Mat image = cv::imread(path, CV_LOAD_IMAGE_COLOR);   // Read the file
                cv::Mat resized;
                

                if (image.size().height > 0) {
                    std::stringstream strs;
                    
                    int score = match._score;
                    int index = match._match_index;
                    int anglePix = match._best_rotation;
		    int angle = angleNormalise(pixelToAngle(anglePix, unwrapRes.width));

                    strs << "Score: " << score << "| Index : " << index << " | Angle : " << angle;
                    
                    cv::Scalar colornum;
                 
                    if( score < 3) {
                        colornum = cv::Scalar(0,255,0);
                    }                          
                    else if( score >= 3 && score < 5) {
                        colornum = cv::Scalar(128,255,0);

                    }                                                    
                    else if (score >= 5 && score < 8) {
                        colornum = cv::Scalar(178,255,102);
                    }                                                 
                    else if (score >= 8 && score < 12) {
                        colornum = cv::Scalar(255,255,0);
                    }                                                 
                    else if (score >= 12 && score < 18) {
                        colornum = cv::Scalar(255,128,0);
                    }                                                     
                    else if (score >= 18){
                        colornum = cv::Scalar(255,0,0);
                    }
                          
                    
                    cv::resize(image,resized, {WIDTH,HEIGHT});
                    cv::putText(
                        resized, 
                        strs.str(),
                        cv::Point(20,20), // Coordinates
                        cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                        0.5, // Scale. 2.0 = 2x bigger
                        colornum, // BGR Color
                        1 // Line Thickness (Optional)
                    );

                    if (score < 18) {
                        cv::imshow("Matched", resized);
                    }
                    
                }
				// TO DO : get distribution of direction angles
				// creating the direction visual 
           
            
        }

        // Show frame difference
        
        cv::resize(outputImage,outputImage, {WIDTH,HEIGHT});
        
       
        
        cv::imshow("Unwrapped", outputImage);
        if(cv::waitKey(1) == 27) {
            break;
        }
	
	cv::waitKey(1);
	//std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (mode == 0) { 
        datab.saveImages();
        datab.saveTrainingData();
        std::cout << "training data saved to disk" << std::endl;
    } 
    
        
    

    return EXIT_SUCCESS;
}


