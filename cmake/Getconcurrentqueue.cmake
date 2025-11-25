include(FetchContent)

FetchContent_Declare(
  concurrentqueue
  GIT_REPOSITORY    https://github.com/cameron314/concurrentqueue.git
  GIT_TAG           master
  SOURCE_DIR "${DEPS_DIR}/concurrentqueue"
)

FetchContent_MakeAvailable(concurrentqueue)
