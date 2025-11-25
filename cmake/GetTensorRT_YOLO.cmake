include(ExternalProject)
ExternalProject_Add(TensorRT_YOLO_external
    GIT_REPOSITORY https://github.com/laugh12321/TensorRT-YOLO.git
    GIT_TAG main
    UPDATE_COMMAND
        ""
    SOURCE_DIR 
        "${DEPS_DIR}/TensorRT_YOLO"
    DOWNLOAD_NO_PROGRESS
        0
    CMAKE_ARGS
        -DTENSORRT_PATH=${TENSORRT_PATH}
        -DTENSORRT_INCLUDE_DIR=${TENSORRT_PATH}/include
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
        -DCMAKE_CXX_EXTENSIONS=${CMAKE_CXX_EXTENSIONS}
        -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY="${DEPS_DIR}/TensorRT_YOLO/lib"
        -DCMAKE_LIBRARY_OUTPUT_DIRECTORY="${DEPS_DIR}/TensorRT_YOLO/lib"
        -DCMAKE_CXX_STANDARD_REQUIRED=${CMAKE_CXX_STANDARD_REQUIRED}
    INSTALL_COMMAND
        ""
)
add_library(TensortRT_YOLO INTERFACE)
add_dependencies(TensortRT_YOLO TensorRT_YOLO_external)
target_include_directories(TensortRT_YOLO
    INTERFACE
        "${DEPS_DIR}/TensorRT_YOLO/include"
)
target_link_libraries(TensortRT_YOLO
    INTERFACE
        "${DEPS_DIR}/TensorRT_YOLO/lib/libdeploy.so"
        "${DEPS_DIR}/TensorRT_YOLO/lib/plugin/libcustom_plugins.so"
)
target_link_libraries(TensortRT_YOLO
    INTERFACE
        CUDA_tensortRT_interface
)