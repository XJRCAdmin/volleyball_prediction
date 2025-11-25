#include "infer_core_wrapperv8.h"

utils::InitParameter SetParam()
{
	utils::InitParameter initParameters;
	initParameters.class_names = utils::dataSets::threeball;
	initParameters.num_class = 1;
	initParameters.src_w = 640;
    initParameters.src_h = 480;
	initParameters.batch_size = 1;
    initParameters.dst_w = 640;
	initParameters.dst_h = 640;
	initParameters.input_output_names = { "images",  "output0" };
	initParameters.conf_thresh = 0.5f;
	initParameters.iou_thresh = 0.45f;
	initParameters.save_path = "";
	return initParameters;
}

InferCoreYolov8Wrapper::InferCoreYolov8Wrapper(std::string file_path, spdlog::logger &logger)
    : yolo(SetParam())
    , logger(logger)
{
    const auto & trt_file = lazy_singleton<std::vector<unsigned char>>::instance(utils::loadModel(file_path));
    std::lock_guard<std::mutex> lck(init_mutex);
    if (trt_file.empty()){
        logger.error("trt_file is empty!");
        throw std::runtime_error("trt_file is empty");
    }
    // init model
    if (!yolo.init(trt_file))
    {
        logger.error("initEngine() ocur errors!");
        throw std::runtime_error("initEngine() ocur errors!");
    }
    logger.info("init trtfile succeed\n");
}

InferResultType InferCoreYolov8Wrapper::infer(cv::Mat &&img)
{
    // auto start = std::chrono::steady_clock::now();
    imgsBatch.clear();
    imgsBatch.emplace_back(img);
    yolo.reset();
    yolo.copy(imgsBatch);
    yolo.preprocess(imgsBatch);
    yolo.infer();
    yolo.postprocess(imgsBatch);
    // auto end = std::chrono::steady_clock::now();
    // logger.info("infer takes {}ms", std::chrono::duration<float, std::milli>(end - start).count());
    auto rslt_in_original_type = yolo.getObjectss();
    return InferResultType{
        .detections= BoxesToDetections(rslt_in_original_type[0]),
        .frame = img
    };
}

std::vector<Detection> InferCoreYolov8Wrapper::BoxesToDetections(std::vector<utils::Box> &OEF)
{
    std::vector<Detection> detections;
    /*
        OEF: Object Each Frame
        objects are wrapped in two vector
        the first vector: "std::vector<utils::Box>"(OEF) is the objects tjat appeared in each frame in "img_batches" (updated in main cpp code)
    */
    for(const auto &obj: OEF){
        detections.emplace_back(
            (Detection){
            cv::Rect_<float>(obj.left, obj.top, (obj.right-obj.left), (obj.bottom-obj.top)), 
            obj.label, 
            obj.confidence
        });
    }
    return detections;
}

std::mutex InferCoreYolov8Wrapper::init_mutex;
