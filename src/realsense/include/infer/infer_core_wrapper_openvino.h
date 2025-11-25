#pragma once 
#include "infer_core_wrapper.h"
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp> 
#include "lazy_singleton.hpp"

class InferCoreOpenvinoWrapper: public InferCoreWrapper
{
static std::mutex init_mutex;
public:
    InferCoreOpenvinoWrapper(std::string file_path, spdlog::logger &logger);
    InferResultType infer(cv::Mat &&img);
    std::vector<Detection> BoxesToDetections(std::vector<utils::Box> &OEF);
private:
    ov::Core core;
    
};


