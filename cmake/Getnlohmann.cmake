include(FetchContent)

set(FETCHCONTENT FALSE)
FetchContent_Declare(
    nlohmann
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.10.5
    SOURCE_DIR "${DEPS_DIR}/nlohmann"
)

# set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
message(STATUS "______ fetch deps and configure _____")
    FetchContent_MakeAvailable(nlohmann_json)
message(STATUS "_____ finish ______")

add_library(nlohmann_json INTERFACE)

target_link_libraries(nlohmann_json
    INTERFACE
    nlohmann_json::nlohmann_json
)
