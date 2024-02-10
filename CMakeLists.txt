cmake_minimum_required(VERSION 3.16)
project(yolov8_ros)

# Find CMake dependencies
find_package(ament_cmake REQUIRED)

# Install python dependencies
add_custom_target(
    poetry_virtualenv ALL
    COMMAND poetry install --no-root --no-ansi --no-interaction
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Building ${CMAKE_PROJECT_NAME} virtualenv... may take some time"
)

ament_package()
