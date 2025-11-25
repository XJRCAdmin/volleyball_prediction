cmake_minimum_required(VERSION 3.22)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME} VERSION 1.0 LANGUAGES CXX)

# cpp standard 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_library(yolo_rt INTERFACE)
target_include_directories(yolo_rt
    INTERFACE "include/infer"
    INTERFACE "include/utils"
)
target_link_libraries(yolo_rt
    INTERFACE OpenCV_interface
    INTERFACE botsort
    INTERFACE fmt_spdlog
)

add_library(yolov8_rt)
target_sources(yolov8_rt
    PRIVATE "src/infer/infer_core_wrapper_v8.cpp"
    PRIVATE "src/infer/infer_core_wrapper.cpp"
)
target_link_libraries(yolov8_rt
    PUBLIC yolo_rt
    PUBLIC tensorrt_yolov8
)
# 去掉该目标编译时的警告
target_compile_options(yolov8_rt PRIVATE "-w")

# add_library(yolov11_rt)
# target_sources(yolov11_rt
#     PRIVATE "src/infer_core_wrapper_v11.cpp"
# )
# target_link_libraries(yolov11_rt
#     PUBLIC yolo_rt
#     PUBLIC TensortRT_YOLO
# )

# add_library(yolov8_ov)
# target_sources(yolov8_ov
#     PRIVATE "src/infer/infer_core_wrapper_openvino.cpp"
#     PRIVATE "src/infer/infer_core_wrapper.cpp"
# )
# target_link_libraries(yolov8_ov
#     PUBLIC yolo_rt
# )
# target_include_directories(yolov8_ov
#     PRIVATE ""
# )
# target_link_directories(yolov8_ov
#     PRIVATE ""
# )

add_library(rstools)
target_include_directories(rstools
    PRIVATE "include/rs"
    PRIVATE "include/utils"
)
target_sources(rstools
    PRIVATE "src/utils/tools.cpp"
    PRIVATE "src/rs/rstools.cpp"
    PRIVATE "src/rs/rs.cpp"
    PRIVATE "src/rs/rotation_estimator.cpp"
)
target_link_libraries(rstools
    PRIVATE fmt_spdlog
    PRIVATE eigen3_interface
    PRIVATE OpenCV_interface
)
add_library(can)
target_include_directories(can
    PRIVATE "include/can"
)
target_sources(can
    PRIVATE "src/can/can.cpp"
)
add_library(pred_ball)
target_include_directories(pred_ball
    PRIVATE "include/pred"
)
target_sources(pred_ball
    PRIVATE PRIVATE "src/pred/pred.cpp"
)
target_link_libraries(pred_ball
    PRIVATE fmt_spdlog
    PRIVATE eigen3_interface
    PRIVATE OpenCV_interface
)
add_executable(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    PRIVATE "include/can"
    PRIVATE "include/infer"
    PRIVATE "include/rs"
    PRIVATE "include/utils"
    PRIVATE "include/pred"
)
target_sources(${PROJECT_NAME}
    PRIVATE "src/main.cpp"
)
target_link_libraries(${PROJECT_NAME} 
    PRIVATE fmt_spdlog
    PRIVATE librealsense2_interface
    PRIVATE eigen3_interface
    PRIVATE OpenCV_interface
    PRIVATE rstools
    PRIVATE yolov8_rt
    PRIVATE serial_port
    PRIVATE can
    # PRIVATE yolov11_rt
    PRIVATE botsort
    PRIVATE pred_ball
)