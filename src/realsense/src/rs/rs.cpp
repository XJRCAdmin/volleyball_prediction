#include "rs.h"

std::mutex World_pos_lck;
static float World_pos[2] = {};
constexpr float rltv_camera_world[3] = {0, -0.28, 0.50};

rstrans::rstrans(): aligned(RS2_STREAM_COLOR) {};

rstrans& rstrans::init(rs2::stream_profile color, rs2::stream_profile depth)
{
    #define AS_VID_PROF .as<rs2::video_stream_profile>()
    color_width = color AS_VID_PROF .width();
    color_height = color AS_VID_PROF .height();
    depth_width = depth AS_VID_PROF .width();
    depth_height = depth AS_VID_PROF .height();
    #undef AS_VID_PROF
    return *this;
}
rstrans& rstrans::init(rs2::frame color, rs2::frame depth_frame)
{
    #define GET_VID_PROF .get_profile().as<rs2::video_stream_profile>()
    color_width = color GET_VID_PROF .width();
    color_height = color GET_VID_PROF .height();
    depth_width = depth_frame GET_VID_PROF .width();
    depth_height = depth_frame GET_VID_PROF .height();
    #undef GET_VID_PROF
    return *this;
}
rstrans& rstrans::init(float _depth_scale)
{
    depth_scale = _depth_scale;
    return *this;
}
rstrans& rstrans::init(rs2::pipeline_profile &profile)
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
    cv::Mat rstrans::get_color_cvmat(rs2::frame &color)
    {
        return cv::Mat(cv::Size(color_width, color_height), CV_8UC3, (void*)color.get_data());
    }

    /***
     * check coord range of color pixel --> 0
     * check coord of depth pixel --> 1
    */
    void rstrans::coord_check(const float pixel_point[2], int type)
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
        if(pixel_point[0] < 0 || pixel_point[0] >= width || pixel_point[1] < 0 || pixel_point[1] >= height){
            std::string errmsg = fmt::format(
                "deproject_pixel_to_point() invalid argumnets: arg(x:{}, y:{}) out of range([0:{},0:{}])",
                pixel_point[0], pixel_point[1], width, height
            );
            throw std::runtime_error{errmsg};
        }
    }
    bool rstrans::in_coord(const float pixel_point[2], int type)
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
    void rstrans::align(rs2::frameset &fs)
    {
        fs = aligned.process(fs);
        std::call_once(init_intrinsics, [&fs,this]{
            aligned_intrin = fs.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
        });
    }

/// 球半径，单位是m
// declared in rs.h
// constexpr float ball_radias = 0.123f;
// constexpr float ball_filter_threshold = 0.7f;
float pow2(float x) { return x * x; }

// according to librealsense-master src/rs.cpp 
float _get_ball_depth_pure_visual(
    rstrans &tr,
    const float pixel_point[2],
    const float rect_width,
    const float rect_height)
{
    float pixel_radius = 0.25f * (rect_height + rect_width);
    float pixel[2][2], points[2][3] = {};
    pixel[0][0] = pixel_point[0] - pixel_radius;
    pixel[0][1] = pixel_point[1] - pixel_radius;
    pixel[1][0] = pixel_point[0] + pixel_radius;
    pixel[1][1] = pixel_point[1] + pixel_radius;
    rs2_deproject_pixel_to_point(points[0], &tr.aligned_intrin, pixel[0], 1.0f);
    rs2_deproject_pixel_to_point(points[1], &tr.aligned_intrin, pixel[1], 1.0f);
    //printf("dis:%f\n",sqrt(pow2(points[0][0]-points[1][0]) + pow2(points[0][1]-points[1][1])));
    float depth = (sqrt(2.0f)*2.0f*ball_radias) / sqrt(pow2(points[0][0]-points[1][0]) + pow2(points[0][1]-points[1][1]));
    return depth;
}

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
    if(center_depth > 3.99f || center_depth <= 0.4f)
        center_depth = _get_ball_depth_pure_visual(tr, pixel_point, rect_width, rect_height);
    if(center_depth <= 0.3f)
    {
        printf("Error: get_ball_coord() meet an unrecoverable error \n");
        return false;
    }
    rs2_deproject_pixel_to_point(center3d, &tr.aligned_intrin, pixel_point, center_depth);
    // printf("(%f, %f)(%f %f %f) d:%f",pixel_point[0], pixel_point[1], center3d[0], center3d[1], center3d[2], center_depth);
    memcpy(sidex3d, center3d, sizeof(center3d));
    memcpy(sidey3d, center3d, sizeof(center3d));
    sidex3d[0] += consider_radias;
    sidey3d[1] += consider_radias;
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
            if(temp3d[2] >= 0.35f)
            {
                count ++ ;
                distance_sum += temp3d[2] + sqrt(pow2(ball_radias) - pow2(temp3d[0] - center3d[0]) - pow2(temp3d[1] - center3d[1]));
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
    float times=sqrt(pow(ball_coord[0], 2)+pow(ball_coord[1], 2)+pow(ball_coord[2], 2)),
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

Eigen::Vector3f GetWorldPosOfCamera()
{
    std::lock_guard<std::mutex> grd(World_pos_lck);
    return Eigen::Vector3f{
        World_pos[0]+rltv_camera_world[0],
        World_pos[1]+rltv_camera_world[1],
        rltv_camera_world[2]};
}

void SetWorldPosOfRobot(float x, float y)
{
    std::lock_guard<std::mutex> grd(World_pos_lck);
    World_pos[0] = x;
    World_pos[1] = y;
}
void rsfilters::filter(rs2::depth_frame &depth_frame)
{
    depth_frame = thr_filter.process(depth_frame);
    depth_frame = temp_filter.process(depth_frame);
}


RSD435i::RSD435i(std::string serial_number, pipe_utility::SIMO<CameraNode> &node_video)
    : node_video(node_video)
    , start_cnt(0)
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
        for(const auto &sensor: device.query_sensors())
        {
            // 更改摄像头的参数设置，使得全局时间戳被启用
            if(sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
                sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0);
            // 使用高填充率预设
            // https://dev.intelrealsense.com/docs/d400-series-visual-presets
            // https://www.intel.com/content/www/us/en/support/articles/000028416/emerging-technologies/intel-realsense-technology.html
            // 
            if(sensor.supports(RS2_OPTION_VISUAL_PRESET))
                sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
        }
    // }
}
void RSD435i::start()
{
    auto profile = pipe.start([this](const rs2::frame frame){
        // printf("frame arrived\n");
        // With callbacks, all synchronized stream will arrive in a single frameset
        if(rs2::frameset fs = frame.as<rs2::frameset>()){
            // 第n次由图像来到时，需要做的事情
            if (start_cnt <= 2)
            {
                start_sys_time = std::chrono::steady_clock::now();
                start_time_stamp = frame.get_timestamp();
                rstranser.init(fs.get_color_frame(), fs.get_depth_frame());
                start_cnt ++;
                init_y = algo.get_theta().y;
                init_z = algo.get_theta().z+3.1415926f/2;
                return ;
            }
            fs.keep();
            node_video.push(std::tuple(
                std::ref(*this), fs, get_posture_vec(), GetWorldPosOfCamera()
            ));
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
Eigen::Vector3f RSD435i::get_posture_vec()
{
    auto rslt = algo.get_theta();
    return Eigen::Vector3f(-rslt.z - 3.1415926f/2, -rslt.x, -(rslt.y-init_y));
    // return  Eigen::AngleAxisf(-rslt.z - 3.1415926f/2, Eigen::Vector3f::UnitX()) *
    //         Eigen::AngleAxisf(-rslt.x, Eigen::Vector3f::UnitY()) *
    //         Eigen::AngleAxisf(-(rslt.y-init_y), Eigen::Vector3f::UnitZ());
}
double RSD435i::get_deivce_duration_from_start(const rs2::frame &frame)
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
sys_time_point_type RSD435i::get_start_sys_time()
{
    return start_sys_time;
}
