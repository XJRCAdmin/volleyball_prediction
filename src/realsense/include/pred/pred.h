#ifndef SORT_H
#define SORT_H

#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <Eigen/Dense>
#include "BallTracker_3D.h"
#include "EKF.h"

using namespace cv;

struct BallPredictorInputType
{
    Eigen::Vector3f abs_ball_crd;
    double frame_time_stamp;
};

class Predictor
{
    enum class STATE
    {
        INIT_POSE,
        INIT_SPEED,
        PREDICT,
        HANG
    }state;
    Predictor();
    void update(BallPredictorInputType &input);
    Eigen::Vector3f get_falling_point();
};


Eigen::Vector3f cauclate_sort(BallPredictorInputType &input,MyKalmanFilter& kalman_filter);

#endif