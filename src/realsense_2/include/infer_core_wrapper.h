#pragma once 

#include <chrono>
#include "spdlog/spdlog.h"
#include "yolov8/yolov8.h"
#include "BoTSORT.h"
#include "opencv2/opencv.hpp"
#include <cstring>
#include <opencv2/opencv.hpp>

#include "lazy_singleton.hpp"

struct InferResultType
{
    std::vector<Detection> decetions;
    cv::Mat frame;
};

/// @brief Static Interface Class, nerver be implemented
class InferCoreWrapper
{
public:
    std::vector<InferResultType> infer(cv::Mat &&img);
};

class InferCoreYolov8Wrapper: public InferCoreWrapper
{
static std::mutex init_mutex;
public:
    InferCoreYolov8Wrapper(std::string file_path, spdlog::logger &logger);
    InferResultType infer(cv::Mat &&img);
    std::vector<Detection> BoxesToDetections(std::vector<utils::Box> &OEF);
private:
    YOLOV8 yolo;
    std::vector<cv::Mat> imgsBatch;
    spdlog::logger &logger;
};
