#pragma once 

#include <chrono>
#include "spdlog/spdlog.h"
#include "BoTSORT.h"

struct InferResultType
{
    std::vector<Detection> detections;
    cv::Mat frame;
};

/// @brief Static Interface Class, nerver be implemented
class InferCoreWrapper
{
public:
    std::vector<InferResultType> infer(cv::Mat &&img);
};

void plot_detections(cv::Mat &image, std::vector<Detection> &detections);