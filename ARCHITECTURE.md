# Package Organization

This repository is a ROS 2 wrapper around the Ultralytics YOLOv8 library. As is
standard with learning-based Python packages, a virtual environment is used to
avoid Python dependency conflicts with other packages on the system. Virtual
environments appear to be incompatible with the standard ROS 2 Python package
architecture, thus `ament_cmake` is used instead of `ament_python`.

## Virtual Environment Specification

`pyproject.toml`, `poetry.lock`, and `poetry.toml` are used by the Poetry
dependency manager to set up a virtual environment. `pyproject.toml` contains
the editable list of Python dependencies used by the package. In general,
`poetry.lock` and `poetry.toml` do not need to be manually edited.

## `yolov8_ros/`

This folder contains this package's ROS 2 nodes. ROS 2 nodes are implemented as
modules with the suffix `_node` and must contain a `main()` function that can
accept zero arguments. Nodes are not meant to be imported by other Python
modules. `parameters.py` contains the sets of ROS 2 parameters declared by each
node and their accompanying documentation.

## ROS 2 Files

### Package and Build System Files

`package.xml` is a required ROS 2 package specification file.

`CMakeLists.txt` defines where files and folders in this repository are copied
or linked during the ROS 2 workspace build and install process.

### Runtime Files

`launch/` contains ROS 2 launch files.

`scripts/python_entrypoint.py` is an internal script used to run Python modules
within the virtual environment from launch files. See additional documentation
at the top of this file.

`rviz2/` contains saved configuration files for the `rviz2` tool.
