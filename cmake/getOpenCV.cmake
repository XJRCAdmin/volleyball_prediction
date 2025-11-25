find_package(OpenCV REQUIRED)
add_library(OpenCV_interface INTERFACE)
target_include_directories(OpenCV_interface
    INTERFACE
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(OpenCV_interface
    INTERFACE
        ${OpenCV_LIBS}
)