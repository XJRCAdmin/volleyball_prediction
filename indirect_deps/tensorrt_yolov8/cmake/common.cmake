# set
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
list(APPEND ALL_LIBS 
  ${CUDA_LIBRARIES} 
  ${CUDA_cublas_LIBRARY}
  ${CUDA_nppc_LIBRARY} ${CUDA_nppig_LIBRARY} ${CUDA_nppidei_LIBRARY} ${CUDA_nppial_LIBRARY})

# include cuda's header
list(APPEND INCLUDE_DIRS ${CUDA_INCLUDE_DIRS})
# message(FATAL_ERROR "CUDA_npp_LIBRARY: ${CUDA_npp_LIBRARY}")


find_library(TRT_NVINFER NAMES nvinfer HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
find_library(TRT_NVINFER_PLUGIN NAMES nvinfer_plugin HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
find_library(TRT_NVONNX_PARSER NAMES nvonnxparser HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
find_library(TRT_NVCAFFE_PARSER NAMES nvcaffe_parser HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
find_path(TENSORRT_INCLUDE_DIR NAMES NvInfer.h HINTS ${TensorRT_ROOT} PATH_SUFFIXES include)
list(APPEND ALL_LIBS ${TRT_NVINFER} ${TRT_NVINFER_PLUGIN} ${TRT_NVONNX_PARSER} ${TRT_NVCAFFE_PARSER})

# include tensorrt's headers
list(APPEND INCLUDE_DIRS ${TENSORRT_INCLUDE_DIR})

# include tensorrt's sample/common headers
set(SAMPLES_COMMON_DIR "${TensorRT_ROOT}/samples/common")
list(APPEND INCLUDE_DIRS ${SAMPLES_COMMON_DIR})
