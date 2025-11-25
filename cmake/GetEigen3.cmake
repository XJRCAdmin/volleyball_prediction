set_property(DIRECTORY PROPERTY EP_BASE ${CMAKE_BINARY_DIR}/subprojects)
set(STAGED_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/deps)

set(Eigen3_MINIMUM_REQUIRED 3.3.0)

find_package(Eigen3 ${Eigen3_MINIMUM_REQUIRED} QUIET)
add_library(eigen3_interface INTERFACE)

if(TARGET Eigen3::Eigen)
  message(STATUS "Found Eigen3 version ${EIGEN3_VERSION_STRING}")
else()
  message(STATUS "Eigen3 could not be located, Building Eigen3 3.4.0 instead.")
  message(STATUS "Eigen3 staged install: ${STAGED_INSTALL_PREFIX}")

  # win: certutil -hashfile .\eigen-3.4.0.zip "SHA256"
  # URL 
  #   https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
  # URL_HASH 
  #   SHA256=??
  # GIT_REPOSITORY
  #   https://gitlab.com/libeigen/eigen.git
  # GIT_TAG
  #   3.4.0
  include(ExternalProject)
  ExternalProject_Add(eigen3_external
    GIT_REPOSITORY
      https://gitlab.com/libeigen/eigen.git
    GIT_TAG
      3.4.0
    DOWNLOAD_EXTRACT_TIMESTAMP
      NEW
    DOWNLOAD_NO_PROGRESS
      0
    UPDATE_COMMAND
      ""
    CONFIGURE_COMMAND
      ${CMAKE_COMMAND} -B build -S . -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}/Eigen
    BUILD_COMMAND
      ${CMAKE_COMMAND} --install build --config Release
    LOG_BUILD
      1
    BUILD_IN_SOURCE
      1
    INSTALL_COMMAND
      ""
    LOG_INSTALL
      1
    BUILD_ALWAYS
      1
  )

  set(
    EIGEN3_ROOT ${STAGED_INSTALL_PREFIX}/Eigen
    CACHE PATH "Path to internally built Eigen3 installation root"
    FORCE
  )
  set(
    Eigen3_INCLUDE_DIR ${EIGEN3_ROOT}/include
    CACHE PATH "Path to internally built Boost include directories"
    FORCE
  )
  add_dependencies(eigen3_interface eigen3_external)
  # Unset internal variables
  unset(_build_byproducts)
endif()


find_package(OpenMP REQUIRED)
find_package(Eigen3 ${Eigen3_MINIMUM_REQUIRED} QUIET CONFIG)
if(TARGET Eigen3::Eigen)
  message(STATUS "Eigen3 v${EIGEN3_VERSION_STRING} found in ${EIGEN3_ROOT}")
  target_include_directories(eigen3_interface
      INTERFACE 
          ${EIGEN3_INCLUDE_DIR}
          "${IDEPS_DIR}/eigen_bindings"
  )
  target_link_libraries(eigen3_interface
      INTERFACE 
          Eigen3::Eigen
  )
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-march=native" _march_native_works)
  check_cxx_compiler_flag("-xHost" _xhost_works)
  if(_march_native_works)
      message(STATUS "Using processor's vector instructions (-march=native compiler flag set)")
      target_compile_options(eigen3_interface
          INTERFACE
              "-march=native"
      )
  elseif(_xhost_works)
      message(STATUS "Using processor's vector instructions (-xHost compiler flag set)")
      target_compile_options(eigen3_interface
          INTERFACE
              "-xHost"
      )
  else()
      message(STATUS "No suitable compiler flag found for vectorization")
  endif()

  find_package(BLAS)
  if(BLAS_FOUND)
      message(STATUS "Eigen will use some subroutines from BLAS.")
      message(STATUS "See: http://eigen.tuxfamily.org/dox-devel/TopicUsingBlasLapack.html")
      target_link_libraries(eigen3_interface
          INTERFACE 
              ${BLAS_LIBRARIES}
      )
      target_compile_definitions(eigen3_interface
          INTERFACE
              EIGEN_USE_BLAS
      )
  else()
      message(STATUS "BLAS not found. Using Eigen own functions")
  endif()
endif()
