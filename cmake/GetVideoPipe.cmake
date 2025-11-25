
FetchContent_Declare(
    VideoPipe
    GIT_REPOSITORY https://github.com/sherlockchou86/VideoPipe.git
    GIT_TAG main
    SOURCE_DIR "${DEPS_DIR}/VideoPipe"
)

message(STATUS "______ fetch deps and configure _____")
    FetchContent_MakeAvailable(VideoPipe)
message(STATUS "_____ finish ______")
