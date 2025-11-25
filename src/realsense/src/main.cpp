
#include <thread>
#include <filesystem>
// for prediction size
#define RUNTIME_CHECK
// ITPF Inferring time per frame
// #define LOG_ITPF
#define LOG_3DPOS
 //#define SHOW_FRAME
#define RECORD_PRED
#define VOLIBALL_MODE_SERIAL 0 /// serial keeps opening
#define VOLIBALL_MODE_CAN    1
#include "rs.h"
#include "infer_core_wrapperv8.h"
#include "serial/FlexSerial.hpp"
#include "can.hpp"
#include "pred.h"

// https://github.com/IntelRealSense/librealsense/blob/master/examples/callback/readme.md
// https://support.intelrealsense.com/hc/en-us/community/posts/360033435954-D435-get-timestamp-of-each-frame
// frames.get_frame_metadata(rs.frame_metadata_value.sensor_timestamp);

int get_thread_index()
{
    static int thread_index = -1;
    static std::mutex m;
    std::lock_guard<std::mutex> lck(m);
    thread_index ++;
    return thread_index;
}

class VideoRecorder
{
public:
    cv::VideoWriter videoWriter;
    VideoRecorder(std::string outputFilename, int fps = 16, int width = 640, int height = 480)
    {
        videoWriter.open(outputFilename, 
                         cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 
                         fps,
                         cv::Size(width, height));
        if (!videoWriter.isOpened()) {
            throw std::runtime_error("Error:Could not open the output video file for write");
        }
    }
    void write(cv::Mat &frame)
    {
        videoWriter.write(frame);
    }
    ~VideoRecorder()
    {
        videoWriter.release();
    }
};

VideoRecorder video_recorder("./video/video.avi");

struct FieldFilter
{
    int x_min, x_max, y_min, y_max;
    FieldFilter()
    {
        x_min = 0;
        x_max = INT_MAX;
        y_min = 0;
        y_max = INT_MAX;
    }
    bool check_in_field(Eigen::Vector3f &abs_pos)
    {
        return abs_pos[0] > x_min && abs_pos[0] < x_max && abs_pos[1] > y_min && abs_pos[1] < y_max;
    }
}field_filter;

rsfilters RSfilter;
MyKalmanFilter kf;
std::mutex mkfm;

#if VOLIBALL_MODE_SERIAL
FlexSerial::Serial ser_up("/dev/ttyUSB0");
#endif
#if VOLIBALL_MODE_CAN
Can::SocketCan can("can0");
#endif

#include "BoTSORT.h"
std::unique_ptr<BoTSORT> tracker = std::make_unique<BoTSORT>("../rtd/config");
struct InferNode
{
    using Input =  std::tuple<RSD435i &, rs2::frameset, Eigen::Vector3f, Eigen::Vector3f>;
    using Output = pipe_utility::end_point;
    using InferCoreType = InferCoreYolov8Wrapper;
    bool first_fall;
    spdlog::logger thread_logger;
    InferCoreType inferer;

    InferNode()
        : thread_logger(tools::make_logger_mt(fmt::format("thrdID:{}", get_thread_index())))
        , inferer{"../rtd/models/trt/volleyball_best.trt", thread_logger}
        , first_fall(false)
    {
        
    }

    void process(Input &input)
    {
        sys_time_point_type start_point = std::chrono::steady_clock::now();
        auto &[camera, frameset, posture_vec, abs_cam_crd] = input;
        //abs_cam_crd = GetWorldPosOfCamera();
        //thread_logger.info("abs_cam_crd: {:.3}", abs_cam_crd);
        if(first_fall)
        {
            return;
        }
        double frame_duration_from_start = camera.get_deivce_duration_from_start(frameset);
        double sys_duration_from_start = (default_duration_type(start_point - camera.get_start_sys_time())).count();
        // if the frame are too old, just ignore it
        if (sys_duration_from_start - frame_duration_from_start > 1000)
        {
            thread_logger.info("frame too old sys_dur:{}ms frame_dur:{}ms ",sys_duration_from_start, frame_duration_from_start);
            return ;
        }
        /// cost:6ms
        camera.rstranser.align(frameset);
        rs2::frame color = frameset.get_color_frame();
        rs2::depth_frame depth_frame = frameset.get_depth_frame();
        
        /// cost:1ms
        RSfilter.filter(depth_frame);
        /// cost:9ms
        auto prediction_result = inferer.infer(camera.rstranser.get_color_cvmat(color));

        /// get ball coord
        // {
        float xy[2] = {}, rltv_cam_crd_arr[3] = {}, conf_thresh = 0.7f;
        std::vector<Detection> alternatives;
        float frame_timestamp = camera.get_deivce_duration_from_start(frameset);
        auto rotate_quaternionf = Eigen::AngleAxisf(posture_vec[0], Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(posture_vec[1], Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(posture_vec[2], Eigen::Vector3f::UnitZ());
        auto reverse_rotation_matrix = rotate_quaternionf.conjugate();
#ifdef LOG_3DPOS
        // std::string frame_det_info = "";
        // fmt::format("frame_timestamp: {:.3} detected_object_num: {}\n", frame_timestamp, prediction_result.detections.size());
        // thread_logger.info("current yaw: {}", camera.algo.get_theta().y);
#endif
        // 通过过滤器的三维坐标的数量
        int filtered_cnt = 0;
        // to find the max conf that fits
        Eigen::Vector3f rslt_rltv_cam_origin_crd, rslt_abs_ball_crd;
        
        float max_conf = 0.0f;
        for (int i = 0; i < prediction_result.detections.size(); i ++)
        {
            bool filtered_out = false;
            const auto & rslt = prediction_result.detections[i];
            xy[0] = rslt.bbox_tlwh.x + 0.5f * rslt.bbox_tlwh.width;
            xy[1] = rslt.bbox_tlwh.y + 0.5f * rslt.bbox_tlwh.height;
            bool ret = get_ball_coord(camera.rstranser, rltv_cam_crd_arr, xy, rslt.bbox_tlwh.width, rslt.bbox_tlwh.height, depth_frame);
            // whether the depth staticstics is robust enougth
            if(rslt.confidence > conf_thresh) {
                // 摄像头坐标系中的坐标 : 
                auto rltv_cam_crd = Eigen::Vector3f(rltv_cam_crd_arr[0],rltv_cam_crd_arr[2],-rltv_cam_crd_arr[1]);
                /// Relative to the camera's origin
                /// 相对于摄像头原点坐标坐标系中的坐标
                Eigen::Vector3f rltv_cam_origin_crd = reverse_rotation_matrix * rltv_cam_crd;
                /// 世界坐标系中的坐标
                //printf("------------abs_cam_crd---%f %f %f\n",abs_cam_crd[0],abs_cam_crd[1],abs_cam_crd[2]);
                Eigen::Vector3f abs_ball_crd = rltv_cam_origin_crd + abs_cam_crd;
                printf("------------abs_ball_crd---%f %f %f\n",abs_ball_crd[0],abs_ball_crd[1],abs_ball_crd[2]);
                // if(field_filter.check_in_field(abs_ball_crd))
                if(true)
                {
                    filtered_cnt ++;
                    if(prediction_result.detections[i].confidence > max_conf)
                    {
                        rslt_rltv_cam_origin_crd = rltv_cam_origin_crd;
                        rslt_abs_ball_crd = abs_ball_crd;
                        max_conf = prediction_result.detections[i].confidence;
                    }
                }
                else
                    filtered_out = true;
            }else
                filtered_out = true;
            if(!filtered_out)
                alternatives.push_back(prediction_result.detections[i]);
        }
        // }
        // std::sort(alternatives.begin(), alternatives.end(), [](const Detection &a, const Detection &b) { return a.confidence > b.confidence; });
        // auto track = tracker->track(prediction_result.detections, prediction_result.frame.clone());
        if(alternatives.size() != 0)
        {
        
#ifdef LOG_3DPOS
            auto frame_det_info = fmt::format("rltv_cam_origin_crd: {:.3} selected abs_ball_crd: {:.3}\n", rslt_rltv_cam_origin_crd, rslt_abs_ball_crd);
            thread_logger.info(frame_det_info);
#endif
#if VOLIBALL_MODE_SERIAL or VOLIBALL_MODE_CAN
            // if(abs_ball_crd[2] < 0.20)
            // {
            //     first_fall = true;
            //     thread_logger.info("----------------------------------------------");
            //     return;
            // }
            float pitch=90, yaw=0;
            auto &cc = rslt_rltv_cam_origin_crd;
            pitch = 90-std::atan(cc[0]/cc[1]) * 180 / 3.1415926 - frame_timestamp*9.36908004e-5;
            yaw = (-camera.init_z+std::atan(cc[2]/ cc[1])) * 180 / 3.1415926;
#ifdef LOG_3DPOS
            thread_logger.info("theta: {:.3} {:.3}", pitch, yaw);
#endif           
            if(pitch > 0 && pitch < 180 && yaw < 70 && yaw > 0)
            {
#if VOLIBALL_MODE_SERIAL
                std::string ser_msg = fmt::format("{:.3} {:.3}!", pitch, yaw);
                ser_up.send(ser_msg);
#endif
#if VOLIBALL_MODE_CAN
                auto wframe = Can::Frame(0x23, 8u).set_data(yaw, pitch);
                can.send(wframe);
                thread_logger.info("can[0x23]:{:.3} {:.3} {:x}\n", yaw, pitch, *(unsigned long long*)(wframe.ptr()->data));
#endif
            }
#endif
            auto ball_predict_input = BallPredictorInputType{
                .abs_ball_crd = rslt_abs_ball_crd,
                .frame_time_stamp = frame_timestamp
            };
            ball_predict(ball_predict_input);
        }
#ifdef LOG_ITPF
        auto end_point = std::chrono::steady_clock::now();
        thread_logger.info("single node process takes {}ms per unit",
            static_cast<std::chrono::duration<float, std::milli>>(end_point - start_point).count()
        );
#endif
        plot_detections(prediction_result.frame, alternatives);
        cv::putText(prediction_result.frame, fmt::format("CP{:.3}", rslt_abs_ball_crd), cv::Point(0, 25), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,255), 2);
        cv::putText(prediction_result.frame, fmt::format("FP{:.3}", _falling_point_pos), cv::Point(0, 50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,255), 2);
        Eigen::Vector3f cam_fp = rotate_quaternionf * Eigen::Vector3f(_falling_point_pos[0], _falling_point_pos[1], -abs_cam_crd[2]);
        float pixel[2] = {}, point[3] = {cam_fp[0], -cam_fp[2], cam_fp[1]};
        rs2_project_point_to_pixel(pixel, &camera.rstranser.aligned_intrin, point);
        // thread_logger.info("pixel:{:.3} {:.3} point:{:.3} {:.3} {:.3}", pixel[0], pixel[1], point[0], point[1], point[2]);
        if(pixel[0] >= 0 && pixel[0] < 640 && pixel[1] >= 0 && pixel[1] < 480)
            cv::rectangle(prediction_result.frame, cv::Rect(pixel[0], pixel[1], 5, 5), cv::Scalar(0,255,0), -1);
#ifdef SHOW_FRAME
        cv::imshow("result", prediction_result.frame);
        cv::waitKey(1);
#endif
#ifdef RECORD_PRED
        // video_recorder.write(prediction_result.frame);
#endif
    }
    void ball_predict(BallPredictorInputType &input)
    {
        std::lock_guard<std::mutex> lck(mkfm);
        // do some calculations to maintain _falling_point_pos

        kf.flash(input.frame_time_stamp);
//        printf("Detect_pos:pos[1]%f,pos[2]%f,pos[3]%f\n",i/nput.abs_ball_crd(0),input.abs_ball_crd(1),input.abs_ball_crd(2));
        Eigen::VectorXf pos=cauclate_sort(input,kf);
        //printf("ballpredict - pos:pos[0]%f,pos[1]%f,pos[1]%f\n",pos[0],pos[1],pos[2] );
        // std::cout<<"ballpredict - pos dimension:"<<pos.size()<<std::endl;
        // for(int i=0;i<pos.size();i++)
        // {
        //     std::cout<<"pos["<<i<<"]"<<pos[i]<<std::endl;
        // }
        {
            std::lock_guard<std::mutex> lck(_read_write_lock);
            // _falling_point_pos should be updated safely here under the guard
            _falling_point_pos << pos[0], pos[1];
            thread_logger.info("predict_pos:{:.3}", _falling_point_pos);
            // update can 
            float posx=pos[0];float posy=pos[1];
            if(!(pos.hasNaN()||!pos.allFinite())){
                auto frame = Can::Frame(0x12, 8u).set_data(posx,posy);
                can.send(frame);
            }
        }
    }
private:
    std::mutex _read_write_lock;
    std::string _modelpath; 
    Eigen::Vector2f _falling_point_pos;
};


auto &RSM = lazy_singleton<rstools::Manage>::instance("rs");

std::atomic<bool> is_running(true);
int main()try
{
    #if VOLIBALL_MODE_CAN == 1
    auto can_recv_thread = std::thread([&]{
        while(is_running)
        {
            auto frame = can.receive();
            if(frame.get_id() == 0x14)
            {
                float posx = 0.001* *(float*)frame.ptr()->data;
                float posy = 0.001* *((float*)(frame.ptr()->data)+1);
                //printf("posx:--------%f,------posy%f\n",posx,posy);
               
                SetWorldPosOfRobot(posx,posy);
            }
        }
    });
    #endif
    /// top level logger for main function
    auto mainlogger = tools::make_logger_mt("main");
    
/// set worker nodes
// {
    auto camera_node = pipe_utility::SIMO<CameraNode>(2u);
    size_t infer_node_thread_num = 2u;
    auto infer_node = pipe_utility::SIMO<InferNode>(camera_node, infer_node_thread_num);
    auto sigpub = pipe_utility::SignalPub{infer_node};
// }


/// initalize and check intelrealsense device    239122071391
// {
    RSD435i d435i_1("233622073892", camera_node);
// }
    
    // wait until all threads are prepared
    while(infer_node.threads_num() < infer_node_thread_num) {std::this_thread::yield();}
    // start threads
    sigpub.pub(pipe_utility::ThreadSignal::START);

/// start device with callbacks
// {
    d435i_1.start();
// }
    mainlogger.info("started");
    while (true)
    {
        std::string ctrl;
        std::cin >> ctrl;
        if(ctrl == "stop")
            sigpub.pub(pipe_utility::ThreadSignal::STOP);
        else if(ctrl == "continue")
            sigpub.pub(pipe_utility::ThreadSignal::START);
        else if(ctrl == "exit")
        {
            is_running = false;
            break;
        }
    }
    #if VOLIBALL_MODE_CAN == 1
        can_recv_thread.join();
    #endif
    spdlog::shutdown();
    return 0;
}
catch (const rs2::error &e)
{
    std::string errmsg = fmt::format("RealSense error calling {}({}):{}", e.get_failed_function(), e.get_failed_args(), e.what());
    std::cerr << errmsg << std::endl;
    RSM.logger.error(errmsg);
    spdlog::shutdown();
    return EXIT_FAILURE;
}
catch(std::exception &e)
{
    std::cerr << e.what() << std::endl;
    spdlog::shutdown();
    return EXIT_FAILURE;
}
catch(...)
{
    std::cerr << "unkown exception" << std::endl;
    spdlog::shutdown();
    return EXIT_FAILURE;
}
