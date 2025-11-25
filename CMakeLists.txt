cmake_minimum_required(VERSION 3.22)

set(CMAKE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(PROJECT_NAME "name_of_the_project")
project(${PROJECT_NAME} VERSION 1.0 LANGUAGES CXX)

# cpp standard 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# cmake policy
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW) # find boost removed
endif()
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW) # find_pakage XxXx_ROOT
endif()

# user defined vars for structure
set(DEPS_DIR "${PROJECT_SOURCE_DIR}/deps")
set(IDEPS_DIR "${PROJECT_SOURCE_DIR}/indirect_deps")
set(DEV_DIR "${PROJECT_SOURCE_DIR}/dev")
set(RUN_DIR "${PROJECT_SOURCE_DIR}/dev/run") # runtime files
set(RTD_DIR "${PROJECT_SOURCE_DIR}/dev/rtd") # runtime deps

# set executable files output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY 	${RUN_DIR})
# set archieve files (.a .lib) output directory
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
# set archieve files (.so .dll) output directory
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

# proxy sets for FetchContent
set(ENV{https_proxy} "http://127.0.0.1:7890")
set(ENV{all_proxy} "http://127.0.0.1:7890")

# set MODE for FetchContent
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
# set(FETCHCONTENT_FULLY_DISCONNECTED ON)

# import defined macros and functions
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(getlibrealsense2)
# include(Getconcurrentqueue)
include(Getfmt_spdlog)
include(GetBoost)
include(GetEigen3)
include(Getserial_port)
set(OpenCV_DIR "~/Projects/yolov8/yolov8/3rdparty/opencv-4.8.0/build")
include(getOpenCV)
# set(TENSORRT_PATH "~/TensorRT-8.5.1.7")
set(TENSORRT_PATH "~/Projects/yolov8/yolov8/3rdparty/TensorRT-8.5.1.7")
include(getCUDA_tensorRT)
include(GetTensorRT_YOLOv8)
include(Getbotsort)






# set(TENSORRT_PATH "/opt/tensorRT/TensorRT-8.6.1.6")
# include(GetTensorRT_YOLO)
include(add_sub_dirs)

ADD_SUB_DIRS(src)
