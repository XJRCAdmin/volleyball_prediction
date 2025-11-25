#include "tools.h"

namespace tools
{
spdlog::logger make_logger_mt(const std::string &loggerName)
{
    auto color_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>
        (std::string("logs/").append(loggerName).append(".log"), true);
    // do extra setting 
    return spdlog::logger(loggerName, {color_sink, file_sink});
}
}
// bind serials https://blog.csdn.net/m0_38144614/article/details/121297159
