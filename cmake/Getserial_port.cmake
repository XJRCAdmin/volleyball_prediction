
include(ExternalProject)
ExternalProject_Add(serial_port_external
  DEPENDS
    boost_interface
  GIT_REPOSITORY
    https://github.com/karthickai/serial.git
  GIT_TAG
    master
  UPDATE_COMMAND
    ""
  SOURCE_DIR 
    "${DEPS_DIR}/serial_port"
  DOWNLOAD_EXTRACT_TIMESTAMP
    NEW
  DOWNLOAD_NO_PROGRESS
    0
  BUILD_COMMAND
    ""
  INSTALL_COMMAND
    ""
)

add_library(serial_port INTERFACE)
add_dependencies(serial_port serial_port_external)
target_include_directories(serial_port
  INTERFACE "${DEPS_DIR}/serial_port/include"
  INTERFACE "${IDEPS_DIR}/serial"
)
target_link_libraries(serial_port
  INTERFACE boost_interface
)
