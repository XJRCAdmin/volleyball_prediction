#pragma once 
#include "yolov8/yolov8.h"
#include "infer_core_wrapper.h"
#include <opencv2/opencv.hpp>
#include "lazy_singleton.hpp"

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
