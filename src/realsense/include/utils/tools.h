#pragma once

#include <cstring>
#include "fmt/format.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

namespace tools
{
    spdlog::logger make_logger_mt(const std::string &loggerName);
    spdlog::logger make_logger_file_mt(const std::string &filename);
}