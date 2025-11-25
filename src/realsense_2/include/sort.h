#ifndef SORT_H
#define SORT_H

#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <Eigen/Dense>
#include "ball_tracker_interface.h"
#include "infer_core_wrapper.h"
#include"BallTracker_3D.h"
#include"EKF.h"

using namespace cv;

Eigen::VectorXf cauclate_sort(BallPredictorInputType &input,MyKalmanFilter& kalman_filter);

#endif