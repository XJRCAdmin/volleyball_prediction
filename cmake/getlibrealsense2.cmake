set_property(DIRECTORY PROPERTY EP_BASE ${CMAKE_BINARY_DIR}/subprojects)
set(STAGED_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/deps)

find_package(realsense2 QUIET)

if("${realsense2_LIBRARY}" STREQUAL "")
message(WARNING "realsense2 SDK not found")
else()
add_library(librealsense2_interface INTERFACE)
target_link_libraries(librealsense2_interface INTERFACE ${realsense2_LIBRARY})
endif()

