cmake_minimum_required(VERSION 3.22)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME} VERSION 1.0 LANGUAGES CXX)

# cpp standard 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_executable(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
    PRIVATE "include"
)
target_sources(${PROJECT_NAME}
    PRIVATE "src/rotation_estimator.cpp"
    PRIVATE "src/tools.cpp"
    PRIVATE "src/rstools.cpp"
    PRIVATE "src/main.cpp"
    PRIVATE "src/infer_core_wrapper.cpp"
    PRIVATE "src/sort.cpp"
)
target_link_libraries(${PROJECT_NAME} 
    PRIVATE fmt_spdlog
    PRIVATE librealsense2_interface
    PRIVATE eigen3_interface
    PRIVATE OpenCV_interface
    PRIVATE tensorrt_yolov8
    PRIVATE botsort
)