#include <cstdio>
#include <numeric>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.h>
#include <librealsense2/rs.hpp>

#include "rstools.hpp"
#include "pipe_utility.hpp"
#include "rotation_estimator.h"

using namespace std::chrono_literals;
using sys_time_point_type = decltype(std::chrono::steady_clock::now());
using default_duration_type = std::chrono::duration<double, std::milli>;

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
    std::once_flag init_intrinsics;

    /***
     * 关于默认构造函数的存在：
     * 在高级API中，相机的内参(intrinsics)与外参(extrinsics)与具体的传感器的具体配置相关联。
     * 因此在stream_config指向性不明确的情况下，只有在pipeline.start()选择了具体配置后，才能确定。
     * 所以，该构造函数必定在pipeline.start()后执行，需要执行移动构造函数，获取与保存参数，在此之后，结构体方能参与运算。
     * 这是一个由于生命周期小于作用域范围的典型的破坏RAII模式的例子。
    */
    rstrans();

    /// 从stream_profile初始化的时候生成参数
    rstrans& init(rs2::stream_profile color, rs2::stream_profile depth);
    /***
    * deceperated, unsafe
    * 从结果图像中获取参数，注意，只需要调用一次即可初始化该参数，而并不是每次生成图像，都要调用。
   */
    rstrans& init(rs2::frame color, rs2::frame depth_frame);
    /// 导入参数
    rstrans& init(float _depth_scale);
    /// 从pipeline.start()的返回值pipeline_profile中获取设备相关的参数。
    rstrans& init(rs2::pipeline_profile &profile);

    /// @brief 将rs2::frame中的图像，以CV8UC3格式读出保存为同等大小的cv::Mat格式
    /// @param <rs2::frame> color 
    /// @return <cv::Mat>
    cv::Mat get_color_cvmat(rs2::frame &color);

    /***
     * check coord range of color pixel --> 0
     * check coord of depth pixel --> 1
    */
    void coord_check(const float pixel_point[2], int type);
    bool in_coord(const float pixel_point[2], int type);
    void align(rs2::frameset &fs);
};

/// 球半径，单位是m
constexpr float ball_radias = 0.123f;
constexpr float ball_filter_threshold = 0.7f;
float pow2(float x);

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
);

struct rsfilters
{
    rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
    rs2::hole_filling_filter hf_filter; // Hole Filling - increase depth frame density
    void filter(rs2::depth_frame &depth_frame);
};

class RSD435i;

struct CameraNode
{
    using Input = pipe_utility::start_point;
    using Output = std::tuple<RSD435i &, rs2::frameset, Eigen::Vector3f, Eigen::Vector3f>;
};

class RSD435i{
public:
    rstools::pipeline pipe;
    rotation_estimator_space::rotation_estimator algo;
    pipe_utility::SIMO<CameraNode> &node_video;
    double start_time_stamp;
    sys_time_point_type start_sys_time;
    int start_cnt;
    rstrans rstranser;
    float init_y, init_z;
    RSD435i(std::string serial_number, pipe_utility::SIMO<CameraNode> &node_video);
    void start();
    Eigen::Vector3f get_posture_vec();
    double get_deivce_duration_from_start(const rs2::frame &frame);
    sys_time_point_type get_start_sys_time();
};

Eigen::Vector3f GetWorldPosOfCamera();
void SetWorldPosOfRobot(float x, float y);
