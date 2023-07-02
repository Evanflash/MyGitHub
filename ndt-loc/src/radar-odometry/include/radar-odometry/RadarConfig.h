#ifndef _RADAR_DATA_CONFIG_H
#define _RADAR_DATA_CONFIG_H

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include <vector>
#include <queue>

namespace radar{

using Mat = cv::Mat;

struct Point {
    float i;
    int r;
    int c;
    Point(float i_, int r_, int c_){
        i = i_;
        r = r_;
        c = c_;
    }
};

// cen2018 Features config
extern int sigma_gauss;
extern float zq;
extern int min_range;

// cen2019 Features config
extern int max_points;



}



#endif