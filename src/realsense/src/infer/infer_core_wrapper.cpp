#include "infer_core_wrapper.h"
void plot_detections(cv::Mat &image, std::vector<Detection> &detections){
    cv::Scalar detection_color = CV_RGB(255,255,255);
    // static std::map<int, cv::Scalar> color_map;
    for (const auto d: detections) {
        auto &bbox_tlwh = d.bbox_tlwh;
        // if(color_map.find(d.class_id) == color_map.end())
        // {
        //     cv::Scalar color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
        //     color_map[d.class_id] = color;
        // }
        cv::rectangle(image,
                        cv::Rect(static_cast<int>(bbox_tlwh.x),
                                static_cast<int>(bbox_tlwh.y),
                                static_cast<int>(bbox_tlwh.width),
                                static_cast<int>(bbox_tlwh.height)),
                        // color_map[d.class_id],
                        cv::Scalar(0,0,255),
                        2);
        // cv::rectangle(image, cv::Rect(10, 10, 20, 20), detection_color, -1);
        // cv::putText(image, "Detection", cv::Point(40, 25), cv::FONT_HERSHEY_SIMPLEX, 0.75, detection_color, 2);
		 cv::putText(image,
                    std::to_string(d.class_id)+"|"+std::to_string(d.confidence),
                    cv::Point(static_cast<int>(bbox_tlwh.x), static_cast<int>(bbox_tlwh.y)),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.75,
                    // color_map[d.class_id],
                    cv::Scalar(0,0,255),
                    2);
    }

}