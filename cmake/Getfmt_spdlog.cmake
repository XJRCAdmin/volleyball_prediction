# get when configure

# fetch deps declarations
include(FetchContent)

Set(FETCHCONTENT_QUIET FALSE)
# add deps: fmt
# doc: https://fmt.dev/11.0/get-started/#cmake
FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 11.0.2
    SOURCE_DIR "${DEPS_DIR}/fmt"
)
# add deps: spdlog 
# doc: https://github.com/gabime/spdlog/wiki/9.-CMake#using-spdlog-in-a-shared-library
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.15.0
    SOURCE_DIR "${DEPS_DIR}/spdlog"
)

set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
message(STATUS "______ fetch deps and configure _____")
    FetchContent_MakeAvailable(fmt)
    FetchContent_MakeAvailable(spdlog)
message(STATUS "_____ finish ______")

# interface libraries
add_library(fmt_spdlog INTERFACE)
target_link_libraries(fmt_spdlog
    INTERFACE
        fmt::fmt
        spdlog::spdlog
)
target_compile_definitions(fmt_spdlog 
    INTERFACE
        SPDLOG_FMT_EXTERNAL
        FMT_HEADER_ONLY=OFF
        SPDLOG_HEADER_ONLY=OFF
)