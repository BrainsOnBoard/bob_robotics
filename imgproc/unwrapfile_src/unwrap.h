#pragma once

#include "common/opencv.h" // used for unwrapping

using namespace std;
using namespace cv;

// parameters for unwrapping
const int unwrap_width = 1920;
const int unwrap_height = 590;
const float cent_x = 0.506944;
const float cent_y = 0.510417;
const float r_inner = 0;
const float r_outer = 0.49;

/* get x and y maps for unwrapping */
void getmaps(Mat &map_x, Mat &map_y, Size ssrc)
{
    // convert relative (0.0 to 1.0) to absolute pixel values
    float ccent_x = round((float) ssrc.width * cent_x);
    float ccent_y = round((float) ssrc.height * cent_y);
    float cr_inner = round((float) ssrc.height * r_inner);
    float cr_outer = round((float) ssrc.height * r_outer);

    // calculate maps
    for (int i = 0; i < unwrap_height; i++) {
        for (int j = 0; j < unwrap_width; j++) {
            float r = ((float) i / (float) unwrap_height) * (cr_outer - cr_inner) + cr_inner;
            float th = M_PI * (-0.5 + 2 * ((float) j / (float) unwrap_width));
            float x = ccent_x + r * sin(th);
            float y = ccent_y + r * cos(th);
            map_x.at<float>(i, j) = x;
            map_y.at<float>(i, j) = y;
        }
    }
}

/* unwrap image im with OpenCV's remap function */
inline void unwrap(Mat &imunwrap, Mat &im, Mat &map_x, Mat &map_y)
{
    remap(im, imunwrap, map_x, map_y, INTER_NEAREST);
}

/* process a single frame (first frame of video, or a single image) */
bool processframe(const char* filepath, Mat &imunwrap, Mat &im, Mat &map_x, Mat map_y)
{
    if (!im.data) {
        cerr << "Error: Could not read data" << endl;
        return false;
    }
    if (im.rows != im.cols) {
        cerr << "File is not panoramic -- skipping " << filepath << endl;
        return false;
    }

    getmaps(map_x, map_y, im.size());

    unwrap(imunwrap, im, map_x, map_y);

    return true;
}
