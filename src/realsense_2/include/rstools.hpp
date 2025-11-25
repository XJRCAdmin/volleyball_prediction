#pragma once

#include <iostream>
#include <exception>
#include <functional>
#include <librealsense2/rs.hpp>
#include "fmt/format.h"
#include "eigen_bindings/formatter.hpp"
#include "tools.h"
#include "lazy_singleton.hpp"

class RSD435i;

namespace rstools
{

class pipeline;

class Manage{
public:
    spdlog::logger logger;
    Manage(const std::string &loggerName);
    void list_devices();
    void list_supported_profiles(const std::string &serialNum);
    rs2::device check_device(const std::string &serialNum);
    rs2::stream_profile get_stream_profile(const std::string &serialNum, rs2_stream stream_type, int stream_index, int width, int height, rs2_format format, int framerate);
    std::string get_info(const rs2::device &dev, rs2_camera_info RS_CAMERA_INFO);
    std::string get_info(const rs2::sensor &dev, rs2_camera_info RS_CAMERA_INFO);
    void set_devices_changed_callback(std::function<void(rs2::event_information info)> cb);
    void reset_devices_changed_callback();
private:
    std::mutex query_lock;
    std::mutex callback_change_lock;
    std::function<void(rs2::event_information info)> device_changed_callback;
    rs2::context ctx;
    rs2::device_list devices;
    friend class pipeline;
};

class pipeline
{
public:
    pipeline();
    template<typename S>
    rs2::pipeline_profile start(S callback)
    {
        rs2::pipeline_profile pipeline_profile;
        try{
            pipeline_profile = _pipeline.start(_config, callback);
        }
        catch (const rs2::error & e) 
        {
            _manager.logger.error("pipeline for device {} start failed", _serial_number);
            // try to check reason for failure:
            check_config();
            throw ;
        }
        return pipeline_profile;
    }

    rs2::device enable_device(const std::string &serial_number);
    rs2::stream_profile enable_stream(rs2_stream stream_type, int stream_index, int width, int height, rs2_format format = RS2_FORMAT_ANY, int framerate = 0);
    rs2::stream_profile enable_stream(rs2_stream stream_type, int width, int height, rs2_format format = RS2_FORMAT_ANY, int framerate = 0);
    rs2::stream_profile enable_stream(rs2_stream stream_type, int stream_index = -1, int framerate = 0);

private:
    Manage &_manager;
    rs2::device _device;
    rs2::pipeline _pipeline;
    rs2::config _config;
    std::string _serial_number;
    bool check_config();
    friend class RSD435i;
};

}