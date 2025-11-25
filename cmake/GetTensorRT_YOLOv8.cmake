
# project( VERSION 1.0 LANGUAGES CXX CUDA)

# require TENSORRT_PATH

# Set C++ CUDA Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 11)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Collect all source files
file(GLOB_RECURSE SOURCES 
    "${IDEPS_DIR}/tensorrt_yolov8/src/*.cpp" "${IDEPS_DIR}/tensorrt_yolov8/src/*.cu"
)

# Create library
add_library(tensorrt_yolov8 INTERFACE)

set(SAMPLES_COMMON_DIR "${TENSORRT_PATH}/samples/common/")
target_include_directories(tensorrt_yolov8 
    INTERFACE "${IDEPS_DIR}/tensorrt_yolov8/include"
    INTERFACE "${SAMPLES_COMMON_DIR}"
)
target_sources(tensorrt_yolov8
    INTERFACE ${SOURCES}
    INTERFACE "${SAMPLES_COMMON_DIR}/logger.cpp"
)
target_link_libraries(tensorrt_yolov8 
    INTERFACE CUDA_tensortRT_interface
    INTERFACE OpenCV_interface
)
target_compile_options(tensorrt_yolov8 INTERFACE "-w")
if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
endif()

