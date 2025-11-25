#include <cstdio>

#include "rstools.hpp"
#include "pipe_utility.hpp"
#include "rotation_estimator.h"
#include "infer_core_wrapper.h"
#include "sort.h"
#include "can.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.h>
#include <thread>

#define RUNTIME_CHECK

#include "rstrans.cpp"

using namespace std::chrono_literals;

// https://github.com/IntelRealSense/librealsense/blob/master/examples/callback/readme.md
// https://support.intelrealsense.com/hc/en-us/community/posts/360033435954-D435-get-timestamp-of-each-frame
// frames.get_frame_metadata(rs.frame_metadata_value.sensor_timestamp);

using sys_time_point_type = decltype(std::chrono::steady_clock::now());
using default_duration_type = std::chrono::duration<double, std::milli>;
CANNode *CANNode::instance = CANNode::get_instance();
std::once_flag CANNode::init_flag;
/*** 为了取代rs2::align 的任务与工作，设计出的坐标转换通用器，其还包含了一些其他辅助功能。
 *   其定义了[空间中的坐标，色彩摄像头空间中的坐标，深度摄像头空间中的坐标]三者互相转化的方法。
 *   用于快速地将一帧图像中的某些特殊像素坐标，获取其对应的深度坐标，与其对应到真实物理世界中的三维坐标。
 *   涉及到的概念与实现的理论依据，见（计算机图形学-基础部分）
 *    
 *   由于参数依赖于摄像头，作为RSD435i对象的可选功能部分存在，生命周期与之相同。
*/
struct rstrans
{
    int color_width, color_height, depth_width, depth_height;
    float depth_scale;
    rs2_intrinsics aligned_intrin;
    rs2::align aligned;
    protected:
    std::once_flag init_intrinsics;
    public:
    /***
     * 关于默认构造函数的存在：
     * 在高级API中，相机的内参(intrinsics)与外参(extrinsics)与具体的传感器的具体配置相关联。
     * 因此在stream_config指向性不明确的情况下，只有在pipeline.start()选择了具体配置后，才能确定。
     * 所以，该构造函数必定在pipeline.start()后执行，需要执行移动构造函数，获取与保存参数，在此之后，结构体方能参与运算。
     * 这是一个由于生命周期小于作用域范围的典型的破坏RAII模式的例子。
    */
    rstrans(): aligned(RS2_STREAM_COLOR) {};

    /// 从stream_profile初始化的时候生成参数
    rstrans& init(rs2::stream_profile color, rs2::stream_profile depth)
    {
        #define AS_VID_PROF .as<rs2::video_stream_profile>()
        color_width = color AS_VID_PROF .width();
        color_height = color AS_VID_PROF .height();
        depth_width = depth AS_VID_PROF .width();
        depth_height = depth AS_VID_PROF .height();
        #undef AS_VID_PROF
        return *this;
    }
    /***
    * deceperated, unsafe
    * 从结果图像中获取参数，注意，只需要调用一次即可初始化该参数，而并不是每次生成图像，都要调用。
   */
    rstrans& init(rs2::frame color, rs2::frame depth_frame)
    {
        #define GET_VID_PROF .get_profile().as<rs2::video_stream_profile>()
        color_width = color GET_VID_PROF .width();
        color_height = color GET_VID_PROF .height();
        depth_width = depth_frame GET_VID_PROF .width();
        depth_height = depth_frame GET_VID_PROF .height();
        #undef GET_VID_PROF
        return *this;
    }
    /// 导入参数
    rstrans& init(float _depth_scale)
    {
        depth_scale = _depth_scale;
        return *this;
    }
    /// 从pipeline.start()的返回值pipeline_profile中获取设备相关的参数。
    rstrans& init(rs2::pipeline_profile &profile)
    {
        auto dev = profile.get_device();
        for(auto &sensor: dev.query_sensors())
        {
            if(auto depth_cam = sensor.as<rs2::depth_sensor>())
            {
                if(depth_cam.supports(RS2_OPTION_DEPTH_UNITS))
                    depth_scale = depth_cam.get_option(RS2_OPTION_DEPTH_UNITS);
                else
                    throw std::runtime_error{"getting runtime option failed"};
            }
            if(auto cam = sensor.as<rs2::color_sensor>()){
                for(const auto& stream: cam.get_active_streams()){
                        printf("%s", fmt::format("       Stream:{} index:{} |{}|{}fps",
                            stream.stream_name(),
                            stream.stream_index(),
                            int(stream.format()),
                            stream.fps()).c_str()
                        );
                }
            }
        }
        return *this;
    }

    /// @brief 将rs2::frame中的图像，以CV8UC3格式读出保存为同等大小的cv::Mat格式
    /// @param <rs2::frame> color 
    /// @return <cv::Mat>
    cv::Mat get_color_cvmat(rs2::frame &color)
    {
        return cv::Mat(cv::Size(color_width, color_height), CV_8UC3, (void*)color.get_data());
    }

    /***
     * check coord range of color pixel --> 0
     * check coord of depth pixel --> 1
    */
    void coord_check(const float pixel_point[2], int type)
    {
        int width, height;
        if(type == 0)
        {
            width = color_width;
            height = color_height;
        }else
        {
            width = depth_width;
            height = depth_height;
        }
        if(pixel_point[0] < 0 || pixel_point[0] >= width || pixel_point[1] < 0 || pixel_point[1] >= height)
        {
            std::string errmsg = fmt::format(
                "deproject_pixel_to_point() invalid argumnets: arg(x:{}, y:{}) out of range([0:{},0:{}])",
                pixel_point[0], pixel_point[1], width, height
            );
            throw std::runtime_error{errmsg};
        }
    }
    bool in_coord(const float pixel_point[2], int type)
    {
        int width, height;
        if(type == 0)
        {
            width = color_width;
            height = color_height;
        }else
        {
            width = depth_width;
            height = depth_height;
        }
        if(pixel_point[0] < 0 || pixel_point[0] >= width || pixel_point[1] < 0 || pixel_point[1] >= height)
            return false;
        return true;
    }
    void show_trincs()
    {
        printf("%d %d %d %d %f\n", color_width, color_height, depth_width, depth_height, depth_scale);
    }
    void align(rs2::frameset &fs)
    {
        fs = aligned.process(fs);
        std::call_once(init_intrinsics, [&fs,this]{
            aligned_intrin = fs.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
        });
    }
    // void deproject_pixel_to_point(float point[3], const float pixel[2], float depth)//原来注释掉
    // {
    //     rs2_deproject_pixel_to_point(point, &aligned_intrin, pixel, depth);
    // }
    // void project_point_to_pixel(float pixel[2], const float point[3])
    // {
    //     rs2_project_point_to_pixel(pixel, &aligned_intrin, point);
    // }
};

/// 球半径，单位是m
constexpr float ball_radias = 0.09f;
constexpr float ball_filter_threshold = 0.7f;
float pow2(float x) { return x * x; }

/// importance reference    
// https://blog.csdn.net/m0_46649553/article/details/121249649


/// algo 通过算法估计出球的中间位置的真实坐标
/// @param[in] pixel_point[2] 模型给出的中心点的坐标
bool get_ball_coord(
    rstrans &tr,
    float ball_coord[3],
    const float pixel_point[2],
    const float rect_width,
    const float rect_height,
    const rs2::depth_frame &depth_frame
){
    static const float consider_radias_rate = 0.5;
    static const float consider_radias = ball_radias * consider_radias_rate;
    static float sidex3d[3], sidex2d[2], sidey3d[3], sidey2d[2], center3d[3], temp3d[3];
    static float pixel_radias_x, pixel_radias_y, pixel[2], distance_sum, count, center_depth;
    static float pixel_right[2], pixel_bottom[2], right3d[3], bottom3d[3];

    tr.coord_check(pixel_point, 0);
    center_depth = depth_frame.get_distance(pixel_point[0], pixel_point[1]);
    if(center_depth<=0.01f)
    {
        std::cout<<"Invalid depth value:"<<center_depth<<"at pixel("<<pixel_point[0]<<","<<pixel_point[1]<<")"<<std::endl;
        return false;
    }
    rs2_deproject_pixel_to_point(center3d, &tr.aligned_intrin, pixel_point, center_depth);
    printf("(%f, %f)(%f %f %f) d:%f",pixel_point[0], pixel_point[1], center3d[0], center3d[1], center3d[2], center_depth);
    memcpy(sidex3d, center3d, sizeof(center3d));
    memcpy(sidey3d, center3d, sizeof(center3d));
    sidex3d[0] += consider_radias;
    sidey3d[1] += consider_radias;
    rs2_project_point_to_pixel(sidex2d, &tr.aligned_intrin, sidex3d);
    rs2_project_point_to_pixel(sidex2d, &tr.aligned_intrin, sidex3d);
    pixel_radias_x = sidex2d[0] - pixel_point[0];
    pixel_radias_y = sidey2d[1] - pixel_point[1];

    distance_sum = 0;
    count = 0;
    for(pixel[0] = std::max(std::ceil(sidex2d[0] - 2 * pixel_radias_x), 0.0f); pixel[0] < sidex2d[0] && pixel[0] < tr.depth_width ; pixel[0] ++)
    for(pixel[1] = std::max(std::ceil(sidey2d[1] - 2 * pixel_radias_y), 0.0f); pixel[1] < sidey2d[1] && pixel[1] < tr.depth_height; pixel[1] ++)
    {
        if(pow2((pixel[0] - pixel_point[0]) / pixel_radias_x) + pow2((pixel[1] - pixel_point[1]) / pixel_radias_y) < 1)
        {
            tr.coord_check(pixel, 1);
            rs2_deproject_pixel_to_point(temp3d, &tr.aligned_intrin, pixel, depth_frame.get_distance(pixel[0], pixel[1]));
            /// if value is valid
            if(temp3d[2] >= 0.2f)
            {
                count ++ ;
                distance_sum += temp3d[2] + pow2(ball_radias) - pow2(temp3d[0] - center3d[0]) - pow2(temp3d[1] - center3d[1]);
            }
        }
    }
    if(count == 0){
        memcpy(ball_coord, center3d, sizeof(center3d));
        
    }else
    {
        memcpy(ball_coord, center3d, sizeof(float)*2);
        ball_coord[2] = distance_sum / count ;
    }
    float bias[3] = {}, times= sqrt(pow(ball_coord[0], 2)+pow(ball_coord[1], 2)+pow(ball_coord[2], 2)),
    cx = ball_coord[0] / times, cy = ball_coord[1] / times, cz = ball_coord[2] / times;
    ball_coord[0] += ball_radias * cx;
    ball_coord[1] += ball_radias * cy;
    ball_coord[2] += ball_radias * cz;
    /// filtering not ball
    /// pre-transpose filter
    // *{
    pixel_right[0] = pixel_point[0];
    pixel_right[1] = pixel_point[1] + rect_width / 2;
    pixel_bottom[0] = pixel_point[0] + rect_height / 2;
    pixel_bottom[1] = pixel_point[1];
    rs2_deproject_pixel_to_point(right3d, &tr.aligned_intrin, pixel_right, center_depth);
    rs2_deproject_pixel_to_point(bottom3d, &tr.aligned_intrin, pixel_bottom, center_depth);
    bool not_a_ball = (right3d[0]- center3d[0]) < (ball_radias * ball_filter_threshold) ||
            (bottom3d[1]-center3d[1]) < (ball_radias * ball_filter_threshold);
    // *}
    return !not_a_ball;
}

struct rsfilters
{
    rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
    rs2::hole_filling_filter hf_filter; // Hole Filling - increase depth frame density
    void filter(rs2::depth_frame &depth_frame)
    {
        depth_frame = thr_filter.process(depth_frame);
        depth_frame = temp_filter.process(depth_frame);
    }
};

class RSD435i;

struct CameraNode
{
    using Input = pipe_utility::start_point;
    using Output = std::tuple<RSD435i &, rs2::frameset, Eigen::Quaternionf>;
};

class RSD435i{
public:
    rstools::pipeline pipe;
    rotation_estimator_space::rotation_estimator algo;
    pipe_utility::SIMO<CameraNode> &node_video;
    double start_time_stamp;
    sys_time_point_type start_sys_time;
    int start_flag;
    rstrans rstranser;
    float init_yaw;
    RSD435i(std::string serial_number, pipe_utility::SIMO<CameraNode> &node_video)
        : node_video(node_video)
        , start_flag(0)
    {
        /// initalize and check intelrealsense device
        // {
            const auto &device = pipe.enable_device(serial_number);
            (void) pipe.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);
            (void) pipe.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
            (void) pipe.enable_stream(RS2_STREAM_GYRO);
            (void) pipe.enable_stream(RS2_STREAM_ACCEL);
            // https://github.com/IntelRealSense/librealsense/blob/master/examples/sensor-control/api_how_to.h
            // 使用~/Projects/yolov8/yolov8/3rdparty/realsenseSDKv2.54.1/build/examples/sensor-control/rs-sensor-control 查看设备支持的选项
            // 更改摄像头的参数设置，使得全局时间戳被启用
            for(const auto &sensor: device.query_sensors())
                if(sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
                    sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0);
        // }
    }
    void start()
    {
        auto profile = pipe.start([this](const rs2::frame frame){
            // printf("frame arrived\n");
            // With callbacks, all synchronized stream will arrive in a single frameset
            if(rs2::frameset fs = frame.as<rs2::frameset>()){
                // 第一次由图像来到时，需要做的事情
                if (start_flag == 0)
                {
                    start_sys_time = std::chrono::steady_clock::now();
                    start_time_stamp = frame.get_timestamp();
                    rstranser.init(fs.get_color_frame(), fs.get_depth_frame());
                    start_flag = 1;
                    return ;
                }
                fs.keep();
                node_video.push(std::tuple(std::ref(*this) , fs, get_rotation_matrix()));
            }else
            // Stream that bypass synchronization (such as IMU) will produce single frames
            {
                // printf("2\n");
                // Cast the frame that arrived to motion frame
                auto motion = frame.as<rs2::motion_frame>();
                // If casting succeeded and the arrived frame is from gyro stream
                if (motion 
                    && motion.get_profile().stream_type() == RS2_STREAM_GYRO
                    && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
                {
                    // Get the timestamp of the current frame
                    double ts = motion.get_timestamp();
                    // Get gyro measures
                    rs2_vector gyro_data = motion.get_motion_data();
                    // Call function that computes the angle of motion based on the retrieved measures
                    algo.process_gyro(gyro_data, ts);
                }
                // If casting succeeded and the arrived frame is from accelerometer stream
                else if (motion 
                    && motion.get_profile().stream_type() == RS2_STREAM_ACCEL 
                    && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
                {
                    // Get accelerometer measures
                    rs2_vector accel_data = motion.get_motion_data();
                    // Call function that computes the angle of motion based on the retrieved measures
                    algo.process_accel(accel_data);
                }
            }
        });
        rstranser.init(profile);
    }
    Eigen::Quaternionf get_rotation_matrix()
    {
        auto rslt = algo.get_theta();
        // printf("%f %f %f\n", rslt.z, -rslt.x, (rslt.y-init_yaw));
        return  Eigen::AngleAxisf(rslt.z + 3.1415926f/2, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(-rslt.x, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf((0), Eigen::Vector3f::UnitZ());
    }
    double get_deivce_duration_from_start(const rs2::frame &frame)
    {
        // if(!frame){
        //     printf("frame is empty\n");
        //     return .0;
        // }
        // if(!frame.supports_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_FRAME_TIMESTAMP))
        // {
        //     printf("frame do not support\n");
        //     return .0;
        // }
        return frame.get_timestamp() - start_time_stamp;
    }
    auto get_start_sys_time()
    {
        return start_sys_time;
    }
};

Eigen::Vector3f GetWorldPosOfCamera()
{
    return Eigen::Vector3f{0, 0, 0};
}

int get_thread_index()
{
    static int thread_index = -1;
    static std::mutex m;
    std::lock_guard<std::mutex> lck(m);
    thread_index ++;
    return thread_index;
}

rsfilters RSfilter;
struct InferNode
{
    using Input =  std::tuple<RSD435i &, rs2::frameset, Eigen::Quaternionf>;
    using Output = pipe_utility::end_point;
    using InferCoreType = InferCoreYolov8Wrapper;
    
    spdlog::logger thread_logger;
    InferCoreType inferer;
    MyKalmanFilter mkf;
    InferNode()
        : thread_logger(tools::make_logger_mt(fmt::format("thrdID:{}", get_thread_index())))
        , inferer{"../rtd/models/trt/basketball.trt", thread_logger}
    {}

    void process(pipe_utility::SIMO<InferNode> &self, Input &input)
    {
        sys_time_point_type start_point = std::chrono::steady_clock::now();
        auto &[camera, frameset, rotate_quaternionf] = input;
        double frame_duration_from_start = camera.get_deivce_duration_from_start(frameset);
        double sys_duration_from_start = (default_duration_type(start_point - camera.get_start_sys_time())).count();
        // if the frameset is too old, just ignore it
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
        float xy[2] = {}, rltv_cam_crd_arr[3] = {}, max_conf = 0.0f;
        std::vector<Eigen::Vector3f> alternatives;
        for (const auto &rslt: prediction_result.decetions)
        {
            xy[0] = rslt.bbox_tlwh.x + 0.5f * rslt.bbox_tlwh.width;
            xy[1] = rslt.bbox_tlwh.y + 0.5f * rslt.bbox_tlwh.height;
            bool ret = get_ball_coord(camera.rstranser, rltv_cam_crd_arr, xy, rslt.bbox_tlwh.width, rslt.bbox_tlwh.height, depth_frame);
            
            // if(ret == 1 && rslt > max_conf)
            // {
                alternatives.push_back(Eigen::Vector3f(rltv_cam_crd_arr[0],rltv_cam_crd_arr[2],-rltv_cam_crd_arr[1]));
            // }
        }
        // /// 摄像头坐标系中的坐标
        // Eigen::Vector3f rltv_cam_crd = Eigen::Vector3f(rltv_cam_crd_arr);
        // thread_logger.info("size:{}", prediction_result.decetions.size());

        // test
        // std::cout<<"process - rotate_quaternionf : w"<<rotate_quaternionf.w()
        //          <<",x"<<rotate_quaternionf.x()
        //          <<",y"<<rotate_quaternionf.y()
        //          <<",z"<<rotate_quaternionf.z()<<std::endl;



        
        for(auto & rltv_cam_crd: alternatives)
        {
            // //Relative to the camera's origin
            // // 相对于摄像头原点坐标坐标系中的坐标
            // test
            // std::cout<<"process - rltv_cam_crd:x"<<rltv_cam_crd.x()
            //          <<",y:"<<rltv_cam_crd.y()
            //          <<",z:"<<rltv_cam_crd.z()<<std::endl;

            Eigen::Vector3f rltv_cam_origin_crd = rotate_quaternionf.conjugate() * rltv_cam_crd;
            thread_logger.info("{:.3} {:.3}", rltv_cam_crd, rltv_cam_origin_crd);
            Eigen::Vector3f abs_cam_crd = GetWorldPosOfCamera();
            BallPredictorInputType rslt{
                .abs_ball_crd = rltv_cam_origin_crd + abs_cam_crd,
                .frame_time_stamp = camera.get_deivce_duration_from_start(frameset)
            };
            ballpredict(rslt, mkf);
        }
        auto end_point = std::chrono::steady_clock::now();
        // thread_logger.info("infer for single framset takes {}ms per unit",
        //     static_cast<std::chrono::duration<float, std::milli>>(end_point - start_point).count()
        // );
    }

    void ballpredict(BallPredictorInputType &input, MyKalmanFilter&kf)
    {
        // do some calculations to maintain _falling_point_pos

        kf.flash(input.frame_time_stamp);
        printf("Detect_pos:pos[1]%f,pos[2]%f,pos[3]%f\n",input.abs_ball_crd(0),input.abs_ball_crd(1),input.abs_ball_crd(2));
        Eigen::VectorXf pos=cauclate_sort(input,kf);
        
        // std::cout<<"ballpredict - pos dimension:"<<pos.size()<<std::endl;
        // for(int i=0;i<pos.size();i++)
        // {
        //     std::cout<<"pos["<<i<<"]"<<pos[i]<<std::endl;
        // }
        {
            std::lock_guard<std::mutex> lck(_read_write_lock);
            // _falling_point_pos should be updated safely here under the guard
            _falling_point_pos << pos[0], pos[1];
            printf("predict_pos:%f %f %f\n", pos[0], pos[1], pos[2]);
            // // update can 
            // CANNode::instance->update_can_data(pos[0], pos[1], pos[2]);

        }
    }
    Eigen::Vector2f get_falling_point()
    {
        std::lock_guard<std::mutex> lck(_read_write_lock);
        return _falling_point_pos;
    }
private:
    std::mutex _read_write_lock;
    Eigen::Vector2f _falling_point_pos;
    std::string _modelpath; 
};


auto &RSM = lazy_singleton<rstools::Manage>::instance("rs");

int main()try
{
    /// top level logger for main function
    auto mainlogger = tools::make_logger_mt("main");
    
/// set worker nodes
// {
    auto camera_node = pipe_utility::SIMO<CameraNode>(2u);
    auto infer_node = pipe_utility::SIMO<InferNode>(camera_node, 1u);
    auto sigpub = pipe_utility::SignalPub{infer_node};
// }


/// initalize and check intelrealsense device
// {
    RSD435i d435i_1("239122071391", camera_node);
// }

    // wait for prepared
    while(infer_node.threads_num() < 2) {std::this_thread::yield();}
    // start threads
    sigpub.pub(pipe_utility::ThreadSignal::START);

/// start device with callbacks
// {
    d435i_1.start();
// }
    mainlogger.info("started");
    while (true)
    {
        // std::string ctrl;
        // std::cin >> ctrl;
        // if(ctrl == "stop")
        //     sigpub.pub(pipe_utility::ThreadSignal::STOP);
        // else if(ctrl == "continue")
        //     sigpub.pub(pipe_utility::ThreadSignal::START);
        // else if(ctrl == "exit")
        //     break;
    }
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

//    required from ‘struct std::_Bind_helper<false,
//     void (VisualInferer::*)(pipe_utility::SIMO<std::tuple<RSD435i&, rs2::frameset, Eigen::Quaternion<float, 0> >,
//     pipe_utility::Endpoint, 3, 0>&, std::tuple<RSD435i&, rs2::frameset, Eigen::Quaternion<float, 0> >), 
//     VisualInferer*,
//     const std::_Placeholder<1>&>’