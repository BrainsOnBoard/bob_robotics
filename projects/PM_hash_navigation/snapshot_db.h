// database
#pragma once

#include <vector>
#include <string> 
#include <iostream> 

#include <iostream>
#include <fstream>


struct Match {

    int _score;
    int _match_index;
    int _best_rotation;

    Match(int score , int match_index ,int best_rotation) :
        _score(score), _match_index(match_index), _best_rotation(best_rotation) {}
};

class Snapshot_DB {
    private:
    std::vector<unsigned long long int> hash_list;
    std::vector<cv::Mat> image_list;
    int currentMatchIndex;
    int last_match_index;
    std::vector<int> list_of_last_matches;

    PM_Hasher m_hasher;
    int current_match;
    
    public:
    Snapshot_DB() : current_match(0) {}
    void addSnapshot(cv::Mat &image) {
        unsigned long long int hash;
        m_hasher.computeDCT_Hash(image, hash);
        hash_list.push_back(hash);          // adding the hash to the list
        image_list.push_back(image);        // adding the image to the list
    }

    void saveImages() {
        int counter = 0;
        for (int i = 0; i < image_list.size(); i++) {   
            std::stringstream ss;
            ss << "training_data/training_image" << i << ".jpg";
            std::string str; 
            ss >> str;
            cv::Mat conv;
            image_list[i].convertTo(conv, CV_8UC3, 255.0); 
         //   imshow("name", image_list[i]);
            cv::waitKey(1);
            bool didWrite = imwrite(str,conv); //write the image to a file as JPEG 
            if (didWrite) counter++;
        }
        std::cout << "successfully saved " << counter << " images " << std::endl;
        
    }

    void saveTrainingData() { // saves hashes
        std::ofstream fileout;
        fileout.open ("training_hashes.txt");
        for (int i = 0; i < hash_list.size(); i++) {
            fileout << hash_list[i] << '\n';  
        }
        fileout.close();
    }

    void loadImages(const char pathOfImages) {
        /* 
        /trainingImages/*.jpg
        /trainingHashes/hash.txt
         */
    }

    void loadTrainingData() { // load hashes
        std::string line;
        std::ifstream filein ("training_hashes.txt");
        if (filein.is_open())
        {
            int counter = 0;



            while ( getline (filein,line, '\n') )
            {
            
                unsigned long long int hashval;
                std::stringstream ss(line); 
                ss >> hashval;
                hash_list.push_back(hashval); 
                std::cout << line << std::endl;
                counter++;
            }


            
            std::cout << counter << " hash values are successfully read " << std::endl;
            filein.close();
        }
    }

    

    static void rollImage(const cv::Mat &imageIn, cv::Mat &imageOut, size_t pixels)
    {
        // Loop through rows
        for (int y = 0; y < imageIn.rows; y++) {
            // Get pointer to start of row
            const float *rowPtr = imageIn.ptr<float>(y);
            float *rowPtrOut = imageOut.ptr<float>(y);
            
            // Rotate row to left by pixels
            std::rotate_copy(rowPtr, rowPtr + pixels, rowPtr + imageIn.cols, rowPtrOut);
        }
    }

    cv::Mat rotatePanoramicImageByAngle(int angle, cv::Mat &imageIn) {
        cv::Size s = imageIn.size();
        int rows = s.height;
        int cols = s.width;

        float divider = 360.0/(float)cols;
        int turnAngle = (int)((float)angle/divider);
        //std::cout << turnAngle << " rows: " << rows << " cols: " << cols <<  std::endl;

        cv::Mat imageOut =  cv::Mat::zeros(cv::Size(cols, rows), CV_64FC1);
        rollImage(imageIn, imageOut, angle);
        return imageOut;
        //return transWrap(imageIn, turnAngle, 0);
    }

    Match findBestMatchRotation(cv::Mat &currentView, cv::Mat &bestRotatedView) {
        unsigned long long int hash;
        m_hasher.computeDCT_Hash(currentView, hash);
        int bestDistance = 1000;
        int bestMatchIndex;
        int bestRotation;
        std::vector<Match> matches;

        std::vector<unsigned long long int> rotationHashes;
        std::vector<cv::Mat> rotatedViews; 
        // get all rotations of the current view
        for (int i= 0; i < currentView.size().width; i++) {
            cv::Mat out = currentView.clone();
            cv::Mat currCopy = currentView.clone();
            rollImage(currCopy, out, i);
            unsigned long long int rotHash;
            m_hasher.computeDCT_Hash(out, rotHash);
            rotationHashes.push_back(rotHash);
            rotatedViews.push_back(out);
        }

        // for all the current rotated view 0-360
        for (int h = 0; h < rotationHashes.size(); h++) {
            // for all the hash in the database
            for (int i = current_match; i < hash_list.size();i++) {
                unsigned long long int current_db_hash = hash_list[i];
                unsigned long long int current_rotation_hash = rotationHashes[h];

                int distance = m_hasher.distance(current_db_hash, current_rotation_hash);
               
                // saving to plot later
                Match possible_match(distance, i, h);
                matches.push_back(possible_match);

                if (distance <= bestDistance) {
                    bestDistance = distance;
                    bestMatchIndex = i;
                    bestRotation = h;
                }
            }
        }

	if (bestDistance <=12) {
	    current_match = bestMatchIndex;
	}
        bestRotatedView = rotatedViews[bestRotation];
        Match bestMatch(bestDistance, bestMatchIndex, bestRotation);
        return bestMatch;
    
    }

    Match findBestMatch(cv::Mat &currentView) {
        unsigned long long int hash;
        m_hasher.computeDCT_Hash(currentView, hash);
        int bestDistance = 1000;
        int bestMatchIndex;
        std::vector<Match> matches;



        std::vector<unsigned long long int> rotationHashes;
        // get all rotations of the current view
        for (int i= 0; i < currentView.size().width; i++) {
            cv::Mat out = currentView;
            rollImage(currentView, out, 1);
            unsigned long long int rotHash;
            m_hasher.computeDCT_Hash(currentView, rotHash);
            rotationHashes.push_back(rotHash);
        }

        for (int i =0; i < hash_list.size();i++) {
            unsigned long long int current_db_hash = hash_list[i];
            int distance = m_hasher.distance(current_db_hash, hash);
            //std::cout << " curr hash " << hash << " dbhash " << current_db_hash << std::endl;

            // saving to plot
            Match possible_match(distance, i, i);
            matches.push_back(possible_match);

            if (distance <= bestDistance) {
                bestDistance = distance;
                bestMatchIndex = i;
                
            }
        }    
   
        int rows = 100;
        cv::Mat distribution(rows, matches.size(), CV_8UC1);
        distribution = 0;
        // checking the score distribution
        for (int i =0; i < matches.size(); i++) {  //cols  
            Match m = matches[i];

      
            int score = m._score;

            if (score > 18) {
                score = 255;
            } else {
                score = score * 5;
            }
            
            for (int j = 0; j < rows; j++) {
                distribution.at<unsigned char>(j, i) = score;
            }
          
        }
        cv::imshow("match distribution", distribution);
        

        // best match is the last one
        Match match(bestDistance, bestMatchIndex, bestMatchIndex);
    
        //std::cout << bestDistance <<  " index = " << bestMatchIndex << std::endl;
        return match;
    }

 

};
