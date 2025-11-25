#include "rstools.hpp"

std::string stream_to_string(rs2_stream stream) {
    switch (stream) {
        case RS2_STREAM_DEPTH: return "Depth";
        case RS2_STREAM_COLOR: return "Color";
        case RS2_STREAM_INFRARED: return "Infrared";
        case RS2_STREAM_FISHEYE: return "Fisheye";
        case RS2_STREAM_ACCEL: return "Accelerometer";
        case RS2_STREAM_GYRO: return "Gyroscope";
        case RS2_STREAM_ANY: return "ANY";
        // 可以继续添加其他流
        default: return "Unknown stream";
    }
}

std::string format_to_string(rs2_format format) {
    switch (format) {
        case RS2_FORMAT_Z16: return "Z16";
        case RS2_FORMAT_DISPARITY16: return "Disparity16";
        case RS2_FORMAT_Z16H: return "Z16H";
        case RS2_FORMAT_RGB8: return "RGB8";
        case RS2_FORMAT_BGR8: return "BGR8";
        case RS2_FORMAT_RGBA8: return "RGBA8";
        case RS2_FORMAT_BGRA8: return "BGRA8";
        case RS2_FORMAT_YUYV: return "YUYV";
        case RS2_FORMAT_RAW8: return "RAW8";
        case RS2_FORMAT_ANY: return "ANY";
        // 可以继续添加其他格式
        default: return "Unknown format";
    }
}


using namespace rstools;

Manage::Manage(const std::string &loggerName)
    : logger(tools::make_logger_mt(loggerName))
{
    devices = ctx.query_devices();
    // ctx.set_devices_changed_callback(
    //     [this](rs2::event_information info){
    //         {
    //             std::lock_guard<std::mutex> guard(query_lock);
    //             devices = ctx.query_devices();
    //         }
    //         {
    //             std::lock_guard<std::mutex> guard(query_lock);
    //             if(device_changed_callback)
    //                 device_changed_callback(info);
    //         }
    //         logger.warn("device connection changed: currently {} device connected", devices.size());
    //     }
    // );
}

void Manage::set_devices_changed_callback(std::function<void(rs2::event_information info)> cb)
{
    std::lock_guard<std::mutex> guard(query_lock);
    device_changed_callback =  cb;
}
void Manage::reset_devices_changed_callback()
{
    std::lock_guard<std::mutex> guard(query_lock);
    device_changed_callback = std::function<void(rs2::event_information info)>();
}

std::string Manage::get_info(const rs2::device &dev, rs2_camera_info RS_CAMERA_INFO)
{
    if(dev.supports(RS_CAMERA_INFO))
        return dev.get_info(RS_CAMERA_INFO);
    else 
        return "Unkown";
}
std::string Manage::get_info(const rs2::sensor &dev, rs2_camera_info RS_CAMERA_INFO)
{
    if(dev.supports(RS_CAMERA_INFO))
        return dev.get_info(RS_CAMERA_INFO);
    else 
        return "Unkown";
}
// https://github.com/IntelRealSense/librealsense/tree/master/tools/enumerate-devices
void Manage::list_devices()
{
    logger.info("Listing Realsense Devices");
    if(devices.size() == 0)
    {
        logger.warn("No device Connected");
        return ;
    }
    logger.info("      Device Name     | Serial Number | Usb Type Descriptor | Locked");
    for(const auto &dev: devices)
    {
        logger.info("{: ^22}|{: ^15}|{: ^21}|{: ^7}",
            get_info(dev, RS2_CAMERA_INFO_NAME),
            get_info(dev, RS2_CAMERA_INFO_SERIAL_NUMBER),
            get_info(dev, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR),
            get_info(dev, RS2_CAMERA_INFO_CAMERA_LOCKED)
        );
        if(get_info(dev, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) == "2.0")
            logger.warn("device connect throgh USB2.0, which may prevent it from working properly.");
        if(get_info(dev, RS2_CAMERA_INFO_CAMERA_LOCKED) == "YES")
            logger.warn("device locked");
    }
}

// https://github.com/IntelRealSense/librealsense/blob/master/examples/sensor-control/api_how_to.h#315
void Manage::list_supported_profiles(const std::string &serialNum)
{
    logger.info("listing supported profiles for device of serial number:{}", serialNum);
    for(int i = 0; i < devices.size(); i ++)
        if(get_info(devices[i], RS2_CAMERA_INFO_SERIAL_NUMBER) == serialNum)
        {
            const auto &dev = devices[i];
            logger.info("Device Number: {} Serial Name:{}",
                get_info(dev, RS2_CAMERA_INFO_NAME),
                get_info(dev, RS2_CAMERA_INFO_SERIAL_NUMBER)
            );
            const auto &sensors = dev.query_sensors();
            for(const auto &sensor : sensors)
            {
                logger.info("   Sensor:{}",
                    get_info(sensor, RS2_CAMERA_INFO_NAME)
                );
                auto stream_profiles = sensor.get_stream_profiles();
                for (auto&& profile : stream_profiles) {
                    if (auto video_stream = profile.as<rs2::video_stream_profile>())
                        logger.info("       Stream:{} index:{} | {}x{} {}fps",
                            video_stream.stream_name(),
                            video_stream.stream_index(),
                            video_stream.width(),
                            video_stream.height(),
                            video_stream.fps()
                        );
                    else if(auto motion_stream = profile.as<rs2::motion_stream_profile>())
                    {
                        logger.info("       Stream:{} index:{} | {}fps",
                            motion_stream.stream_name(),
                            motion_stream.stream_index(),
                            motion_stream.fps()
                        );
                    }
                    else
                        logger.info("       Stream:{}", profile.stream_name());
                }
            }
            return ;
        }
    logger.warn("no device found");
}

rs2::device Manage::check_device(const std::string &serialNum)
{
    for(int i = 0; i < devices.size(); i ++)
        if(get_info(devices[i], RS2_CAMERA_INFO_SERIAL_NUMBER) == serialNum)
            return devices[i];
    std::string errmsg = fmt::format(
        "failed to enable device of serial number {}",
        serialNum
    );
    throw std::runtime_error{errmsg};
    return {};
}

rs2::stream_profile Manage::get_stream_profile(const std::string &serialNum, rs2_stream stream_type, int stream_index, int width, int height, rs2_format format, int framerate)
{
    for(int i = 0; i < devices.size(); i ++)
        if(get_info(devices[i], RS2_CAMERA_INFO_SERIAL_NUMBER) == serialNum)
            for(const auto &sensor : devices[i].query_sensors())
                for (auto&& profile : sensor.get_stream_profiles())
                    if(profile.format() == format || format == RS2_FORMAT_ANY)
                        if (auto video_stream = profile.as<rs2::video_stream_profile>())
                        {
                            if ((stream_type == video_stream.stream_type() || stream_type == RS2_STREAM_ANY) &&
                                (stream_index == video_stream.stream_index() || stream_index == -1) &&
                                (width == video_stream.width()   || width == 0) &&
                                (height == video_stream.height() || height == 0) &&
                                (framerate == video_stream.fps() || framerate == 0))
                                return profile;
                        }
                        else if(auto motion_stream = profile.as<rs2::motion_stream_profile>())
                        {
                            if ((stream_type == motion_stream.stream_type() || stream_type == RS2_STREAM_ANY)&&
                                (stream_index == motion_stream.stream_index() || stream_index == -1) &&
                                width == 0  &&
                                height == 0 &&
                                (framerate == motion_stream.fps() || framerate == 0))
                                return profile;
                        }else if(auto pose_stream = profile.as<rs2::pose_stream_profile>())
                        {
                            if ((stream_type == pose_stream.stream_type() || stream_type == RS2_STREAM_ANY) &&
                                (stream_index == pose_stream.stream_index() || stream_index == -1) &&
                                width == 0  &&
                                height == 0 &&
                                (framerate == pose_stream.fps() || framerate == 0))
                                return profile;
                        }
    std::string errmsg = fmt::format(
        "failed to get stream profiles ({}, {}, {} {}x{} {}fps) of device {}",
        stream_to_string(stream_type), stream_index, format_to_string(format),
        width, height, framerate, serialNum
    );
    throw std::runtime_error{errmsg};
    return {};
}

pipeline::pipeline()
    : _manager(lazy_singleton<rstools::Manage>::instance("rs"))
    , _pipeline(_manager.ctx)
{}


bool pipeline::check_config()
{
    bool ret = _config.can_resolve(_pipeline);
    _manager.logger.info("pipeline support: {}\n", ret ? "True" : "False");
    return ret;
}

rs2::device pipeline::enable_device(const std::string &serial_number)
{
    _serial_number = serial_number;
    try{
        _device = _manager.check_device(serial_number);
    }catch(...){
        std::string errmsg = fmt::format("failed to enable device {}", _serial_number);
        _manager.logger.error(errmsg);
        throw ;
    }
    return _device;
}

rs2::stream_profile pipeline::enable_stream(rs2_stream stream_type, int stream_index, int width, int height, rs2_format format, int framerate)
{
    rs2::stream_profile profile;
    try{
        profile = _manager.get_stream_profile(_serial_number, stream_type, stream_index, width, height, format, framerate);
        _config.enable_stream(stream_type, stream_index, width, height, format, framerate);
    }catch(...){
        std::string errmsg = fmt::format(
            "failed to enable stream ({}, {}, {} {}x{} {}fps) of device {}",
            stream_to_string(stream_type), stream_index, format_to_string(format),
            width, height, framerate, _serial_number
        );
        _manager.logger.error(errmsg);
        throw ;
    }
    return profile;
}

rs2::stream_profile pipeline::enable_stream(rs2_stream stream_type, int width, int height, rs2_format format, int framerate)
{
    return this->enable_stream(stream_type, -1, width, height, format, framerate);
}

rs2::stream_profile pipeline::enable_stream(rs2_stream stream_type, int stream_index, int framerate)
{
    return this->enable_stream(stream_type, stream_index, 0, 0, RS2_FORMAT_ANY, framerate);
}