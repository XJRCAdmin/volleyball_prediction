set_property(DIRECTORY PROPERTY EP_BASE ${CMAKE_BINARY_DIR}/subprojects)
set(STAGED_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/deps)
message(STATUS "Boost staged install: ${STAGED_INSTALL_PREFIX}")

list(APPEND BOOST_COMPONENTS_REQUIRED filesystem system asio serial)
set(Boost_MINIMUM_REQUIRED 1.70.0)

add_library(boost_interface INTERFACE)

find_package(Boost ${Boost_MINIMUM_REQUIRED} QUIET)

if(Boost_FOUND)
  message(STATUS "Found Boost version ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION}")
  foreach(_lib IN LISTS ${BOOST_COMPONENTS_REQUIRED})
    target_link_directories(boost_interface
      INTERFACE Boost::${_lib}
    )
  endforeach()
  target_compile_options(boost_interface
    INTERFACE "-pthread"
  )
else()
  message(STATUS "Boost ${Boost_MINIMUM_REQUIRED} could not be located, Building Boost 1.86.0 instead.")

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    if(APPLE)
      set(_toolset "darwin")
    else()
      set(_toolset "gcc")
    endif()
  elseif(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    set(_toolset "clang")
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
    if(APPLE)
      set(_toolset "intel-darwin")
    else()
      set(_toolset "intel-linux")
    endif()
  endif()

  # Non-empty list. Compiled libraries needed
  if(NOT "${BOOST_COMPONENTS_REQUIRED}" STREQUAL "")
    # Replace unit_test_framework (used by CMake's find_package) with test (understood by Boost build toolchain)
    string(REPLACE "unit_test_framework" "test" _b2_needed_components "${BOOST_COMPONENTS_REQUIRED}")
    # Generate argument for BUILD_BYPRODUCTS
    set(_build_byproducts)
    set(_b2_select_libraries)
    foreach(_lib IN LISTS _b2_needed_components)
      list(APPEND _build_byproducts ${STAGED_INSTALL_PREFIX}/boost/lib/libboost_${_lib}${CMAKE_SHARED_LIBRARY_SUFFIX})
      list(APPEND _b2_select_libraries --with-${_lib})
    endforeach()
    # Transform the ;-separated list to a ,-separated list (digested by the Boost build toolchain!)
    string(REPLACE ";" "," _b2_needed_components "${_b2_needed_components}")
    set(_bootstrap_select_libraries "--with-libraries=${_b2_needed_components}")
    string(REPLACE ";" ", " printout "${BOOST_COMPONENTS_REQUIRED}")
    message(STATUS "  Libraries to be built: ${printout}")
  endif()

  # URL https://archives.boost.io/release/1.86.0/source/boost_1_86_0.zip
  # URL_HASH SHA256=cd20a5694e753683e1dc2ee10e2d1bb11704e65893ebcc6ced234ba68e5d8646
  # GIT_REPOSITORY https://github.com/boostorg/boost.git
  # GIT_TAG boost-1.86.0
  include(ExternalProject)
  ExternalProject_Add(boost_external
    URL 
      https://archives.boost.io/release/1.86.0/source/boost_1_86_0.zip
    URL_HASH 
      SHA256=cd20a5694e753683e1dc2ee10e2d1bb11704e65893ebcc6ced234ba68e5d8646
    DOWNLOAD_EXTRACT_TIMESTAMP
      NEW
    DOWNLOAD_NO_PROGRESS
      0
    UPDATE_COMMAND
      ""
    CONFIGURE_COMMAND
      <SOURCE_DIR>/bootstrap.sh
      --with-toolset=${_toolset}
      --prefix=${STAGED_INSTALL_PREFIX}/boost
      ${_bootstrap_select_libraries}
    BUILD_COMMAND
      <SOURCE_DIR>/b2 -q
           link=shared
           threading=multi
           variant=release
           toolset=${_toolset}
           ${_b2_select_libraries}
    LOG_BUILD
      1
    BUILD_IN_SOURCE
      1
    INSTALL_COMMAND
      <SOURCE_DIR>/b2 -q install
          link=shared
          threading=multi
          variant=release
          toolset=${_toolset}
          ${_b2_select_libraries}
    LOG_INSTALL
      1
    BUILD_BYPRODUCTS
      "${_build_byproducts}"
    BUILD_ALWAYS
      1
  )
  
  set(
    BOOST_ROOT ${STAGED_INSTALL_PREFIX}/boost
    CACHE PATH "Path to internally built Boost installation root"
    FORCE
  )
  set(
    BOOST_INCLUDE_DIR ${BOOST_ROOT}/include
    CACHE PATH "Path to internally built Boost include directories"
    FORCE
  )
  set(
    BOOST_LIBRARY_DIR ${BOOST_ROOT}/lib
    CACHE PATH "Path to internally built Boost library directories"
    FORCE
  )

  # Unset internal variables
  unset(_toolset)
  unset(_b2_needed_components)
  unset(_build_byproducts)
  unset(_b2_select_libraries)
  unset(_boostrap_select_libraries)

  add_dependencies(boost_interface boost_external)
  target_include_directories(boost_interface
    INTERFACE ${BOOST_INCLUDE_DIR}
  )
  target_link_directories(boost_interface
    INTERFACE ${BOOST_LIBRARY_DIR}
  )
  target_compile_definitions(boost_interface
    INTERFACE "-pthreads"
  )

endif()

