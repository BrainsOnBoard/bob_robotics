
#include "imgproc/dct_hash.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>


// Standard C includes
#include <ctime>
#include <chrono>
#include <string.h>





#include "dataset_evaluator.cpp"




#define MAX_SPEED 6.28

#define RESIZED_WIDTH 256// 255
#define RESIZED_HEIGHT 64 // 64

#define PM_RESIZE_FACTOR 1

// All the webots classes are defined in the "webots" namespace

// entry point of the controller
int main(int argc, char **argv) {

    int skipstep = 1;
    int section = 1;
    std::string difference_method = "pixel";
    bool isSequence = false;
    bool isPanoramic = false;
    cv::Size resolution = {256,256};
    std::string db_path = "../../../../../../../../media/tenxlenx/3d_assets/simulation_databases/";
    DatasetEvaluator dse1(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "summer", true, 1);
    /*
    DatasetEvaluator dse2(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "summer", true, 2);
    DatasetEvaluator dse3(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "summer", true, 3);

    DatasetEvaluator dse4(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "spring", true, 1);
    DatasetEvaluator dse5(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "spring", true, 2);
    DatasetEvaluator dse6(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "spring", true, 3);

    DatasetEvaluator dse7(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "winter", true, 1);
    DatasetEvaluator dse8(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "winter", true, 2);
    DatasetEvaluator dse9(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "winter", true, 3);
    */
    //DatasetEvaluator dse(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "ALDERLEY", "summer",  "fall", true, section);
    dse1.score_dataset();
    /*
    dse2.score_dataset();
    dse3.score_dataset();
    dse4.score_dataset();
    dse5.score_dataset();
    dse6.score_dataset();
    dse7.score_dataset();
    dse8.score_dataset();
    dse9.score_dataset();*/

    return 0; //EXIT_SUCCESS
}

