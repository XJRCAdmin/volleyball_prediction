# get when configure

Set(FETCHCONTENT_QUIET FALSE)
FetchContent_Declare(
    botsort
    GIT_REPOSITORY https://github.com/viplix3/BoTSORT-cpp.git
    GIT_TAG main
    SOURCE_DIR "${DEPS_DIR}/botsort"
)

message(STATUS "______ fetch deps and configure _____")
    FetchContent_MakeAvailable(botsort)
message(STATUS "_____ finish ______")

target_link_libraries(botsort CUDA_tensortRT_interface)