
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
#define RESIZED_HEIGHT 256 // 64

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
    //DatasetEvaluator dse(db_path, "big_forest/route1_red", "big_forest/route1_cloudy", resolution,  10, difference_method, true, "WEBOTS", "fall",  "summer", true, 1);
    //dse.score_dataset();


{
    DatasetEvaluator dse1(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "summer", true, 1);
    DatasetEvaluator dse2(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "summer", true, 2);
    DatasetEvaluator dse3(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "summer", true, 3);

    DatasetEvaluator dse4(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "spring", true, 1);
    DatasetEvaluator dse5(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "spring", true, 2);
    DatasetEvaluator dse6(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "spring", true, 3);

    DatasetEvaluator dse7(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "winter", true, 1);
    DatasetEvaluator dse8(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "winter", true, 2);
    DatasetEvaluator dse9(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "fall",  "winter", true, 3);
    dse1.score_dataset();
    dse2.score_dataset();
    dse3.score_dataset();
    dse4.score_dataset();
    dse5.score_dataset();
    dse6.score_dataset();
    dse7.score_dataset();
    dse8.score_dataset();
    dse9.score_dataset();
}
/*
{
    DatasetEvaluator dse10(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "summer",  "fall", true, 1);
    DatasetEvaluator dse11(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "summer",  "fall", true, 2);
    DatasetEvaluator dse12(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "summer",  "fall", true, 3);

    DatasetEvaluator dse13(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "summer",  "spring", true, 1);
    DatasetEvaluator dse14(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "summer",  "spring", true, 2);
    DatasetEvaluator dse15(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "summer",  "spring", true, 3);

    DatasetEvaluator dse16(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "summer",  "winter", true, 1);
    DatasetEvaluator dse17(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "summer",  "winter", true, 2);
    DatasetEvaluator dse18(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "summer",  "winter", true, 3);
    dse10.score_dataset();
    dse11.score_dataset();
    dse12.score_dataset();
    dse13.score_dataset();
    dse14.score_dataset();
    dse15.score_dataset();
    dse16.score_dataset();
    dse17.score_dataset();
    dse18.score_dataset();
}
*/
/*
{
    DatasetEvaluator dse19(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "spring",  "fall", true, 1);
    DatasetEvaluator dse20(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "spring",  "fall", true, 2);
    DatasetEvaluator dse21(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "spring",  "fall", true, 3);
    dse19.score_dataset();
    dse20.score_dataset();
    dse21.score_dataset();
}
{
    DatasetEvaluator dse22(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "spring",  "summer", true, 1);
    DatasetEvaluator dse23(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "spring",  "summer", true, 2);
    DatasetEvaluator dse24(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "spring",  "summer", true, 3);
    dse22.score_dataset();
    dse23.score_dataset();
    dse24.score_dataset();
}
{
    DatasetEvaluator dse25(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "spring",  "winter", true, 1);
    DatasetEvaluator dse26(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "spring",  "winter", true, 2);
    DatasetEvaluator dse27(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "spring",  "winter", true, 3);
    dse25.score_dataset();
    dse26.score_dataset();
    dse27.score_dataset();

}

{
    DatasetEvaluator dse28(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "winter",  "summer", true, 1);
    DatasetEvaluator dse29(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "winter",  "summer", true, 2);
    DatasetEvaluator dse30(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "winter",  "summer", true, 3);
    dse28.score_dataset();
    dse29.score_dataset();
    dse30.score_dataset();
}
*/
{
  //  DatasetEvaluator dse31(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "winter",  "spring", true, 1);
 //   DatasetEvaluator dse32(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "winter",  "spring", true, 2);
  //  DatasetEvaluator dse33(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "winter",  "spring", true, 3);
  //  dse31.score_dataset();
 //   dse32.score_dataset();
  //  dse33.score_dataset();
}
/*
{

    DatasetEvaluator dse34(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "winter",  "fall", true, 1);
    DatasetEvaluator dse35(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "winter",  "fall", true, 2);
    DatasetEvaluator dse36(db_path, "", "", resolution,  skipstep, difference_method, isPanoramic, "NORDLAND", "winter",  "fall", true, 3);
    dse34.score_dataset();
    dse35.score_dataset();
    dse36.score_dataset();
}
*/

    //DatasetEvaluator dse(db_path, "training_image_path", "test_image_path", resolution,  skipstep, difference_method, isPanoramic, "ALDERLEY", "summer",  "fall", true, section);
    /*dse1.score_dataset();
    dse2.score_dataset();
    dse3.score_dataset();
    dse4.score_dataset();
    dse5.score_dataset();
    dse6.score_dataset();
    dse7.score_dataset();
    dse8.score_dataset();
    dse9.score_dataset();
    dse10.score_dataset();
    dse11.score_dataset();
    dse12.score_dataset();
    dse13.score_dataset();
    dse14.score_dataset();
    dse15.score_dataset();
    dse16.score_dataset();
    dse17.score_dataset();
    dse18.score_dataset();
    dse19.score_dataset();
    dse20.score_dataset();
    dse21.score_dataset();
    dse22.score_dataset();
    dse23.score_dataset();
    dse24.score_dataset();
    dse25.score_dataset();
    dse26.score_dataset();
    dse27.score_dataset();
    dse28.score_dataset();
    dse29.score_dataset();
    dse30.score_dataset();
    dse31.score_dataset();
    dse32.score_dataset();
    dse33.score_dataset();
    dse34.score_dataset();
    dse35.score_dataset();
    dse36.score_dataset();*/

    return 0; //EXIT_SUCCESS
}

