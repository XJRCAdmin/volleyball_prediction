cmake_policy(SET CMP0091 NEW)                           # 允许在CMake 3.10+中自动设置项目名作为二进制目录名
cmake_policy(SET CMP0146 OLD)                           # 忽略对find_package的过时警告
project(TensorRT-CUDA VERSION 4.3.0 LANGUAGES CXX CUDA) # 定义项目名称、版本和使用的编程语言（C++和CUDA）

add_library(CUDA_tensortRT_interface INTERFACE)

find_package(CUDA REQUIRED)                             # 查找CUDA包
set(CMAKE_CUDA_ARCHITECTURES native)                    # 自动检测最佳CUDA架构
set(CUDA_PATH ${CUDA_TOOLKIT_ROOT_DIR})                 # 缓存CUDA路径

# 添加CUDA定义、包含目录、链接库
target_compile_definitions(CUDA_tensortRT_interface INTERFACE ${CUDA_DEFINITIONS})
target_include_directories(CUDA_tensortRT_interface INTERFACE ${CUDA_INCLUDE_DIRS})
target_link_libraries(CUDA_tensortRT_interface INTERFACE ${CUDA_cudart_LIBRARY})

# 添加TensorRT的包含目录、库目录、链接库
target_include_directories(CUDA_tensortRT_interface INTERFACE "${TENSORRT_PATH}/include")
target_link_directories(CUDA_tensortRT_interface INTERFACE "${TENSORRT_PATH}/lib")
