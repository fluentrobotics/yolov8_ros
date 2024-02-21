cmake_minimum_required(VERSION 3.16)
project(yolov8_ros)

# Find CMake dependencies
find_package(ament_cmake REQUIRED)

# Build Python virtual environment
add_custom_target(
    poetry_virtualenv ALL
    COMMAND poetry install --no-root --no-ansi --no-interaction
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Building ${CMAKE_PROJECT_NAME} virtualenv... this may take some time"
)

# Install launch files.
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html
install(
    DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

# ROS 1 provided a mechanism in XML launch files to find the corresponding
# source directory of a package. ROS 2 removed this mechanism but instead allows
# users to reference the "install" path of a package within a workspace. Thus,
# the virtual environment in this source directory must be discoverable inside
# the workspace's install directory. We manually symlink the virtual environment
# directory for two reasons:
# 1) "colcon build" would otherwise copy gigabytes worth of libraries
# 2) "colcon build --symlink-install" creates a symlink for each file in the
#    directory, rather than a single symlink for the directory itself.
#
# ROS 2 looks for executables in ${CMAKE_INSTALL_PREFIX}/lib.
file(
    # "file" commands are executed at build time. The install directory won't
    # exist yet during a fresh build, which will cause the next CREATE_LINK to
    # fail.
    MAKE_DIRECTORY
    ${CMAKE_INSTALL_PREFIX}
)
file(
    CREATE_LINK
    ${CMAKE_SOURCE_DIR}/.venv
    ${CMAKE_INSTALL_PREFIX}/.venv
    SYMBOLIC
)
install(
    DIRECTORY
    yolov8_ros
    DESTINATION lib/${PROJECT_NAME}
)
install(
    PROGRAMS
    scripts/python_entrypoint.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
