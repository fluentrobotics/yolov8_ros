# YOLOv8 ROS Integration

This repository contains Python modules and a dependency specification to use
Ultralytics YOLOv8 with ROS 2. Additional documentation about how this
repository is organized can be found in [`ARCHITECTURE.md`](/ARCHITECTURE.md).

## Getting Started

### Requirements

- ROS 2 Humble
- Python 3.10
  - Included with Ubuntu 22.04
  - For other supported Ubuntu LTS distributions, the [Deadsnakes Ubuntu
    PPA][deadsnakes] provides the required `python3.10 python3.10-distutils
    python3.10-venv` packages.
- The [Poetry][poetry-docs] dependency manager
- A CUDA 11.8.x runtime environment

[deadsnakes]: https://launchpad.net/~deadsnakes/+archive/ubuntu/ppa
[poetry-docs]: https://python-poetry.org/docs/

### Install dependencies to a virtual environment

This repository uses Poetry to install dependencies to a virtual environment.
The virtual environment is located at the top level of this repository
regardless of the next command used to initialize it.

If you are developing outside of a ROS 2 workspace, run `poetry install
--no-root` within this repository.

This repository is set up to build with `colcon` using the `ament_cmake`
backend. If this repository is inside a ROS 2 workspace, run `colcon build` from
the top level of the ROS 2 workspace.

### (Optional) Run YOLO Checks

After the virtual environment is initialized, you can run YOLO status checks
using `poetry run yolo check`. You should see output similar to below:

```shell
$ poetry run yolo check
Ultralytics YOLOv8.0.200 🚀 Python-3.10.13 torch-2.1.0+cu118 CUDA:0 (NVIDIA RTX A2000 12GB, 12017MiB)
Setup complete ✅ (24 CPUs, 31.0 GB RAM, 53.8/198.7 GB disk)

OS                  Linux-6.5.0-1004-oem-x86_64-with-glibc2.31
Environment         Linux
Python              3.10.13
Install             git
RAM                 31.03 GB
CPU                 13th Gen Intel Core(TM) i7-13700
CUDA                11.8

matplotlib          ✅ 3.8.0>=3.3.0
numpy               ✅ 1.26.1>=1.22.2
opencv-python       ✅ 4.8.1.78>=4.6.0
pillow              ✅ 10.1.0>=7.1.2
pyyaml              ✅ 6.0.1>=5.3.1
requests            ✅ 2.31.0>=2.23.0
scipy               ✅ 1.11.3>=1.4.1
torch               ✅ 2.1.0+cu118>=1.8.0
torchvision         ✅ 0.16.0+cu118>=0.9.0
tqdm                ✅ 4.66.1>=4.64.0
pandas              ✅ 2.1.1>=1.1.4
seaborn             ✅ 0.13.0>=0.11.0
psutil              ✅ 5.9.6
py-cpuinfo          ✅ 9.0.0
thop                ✅ 0.1.1-2209072238>=0.1.1
```

### Running ROS Nodes

All ROS nodes are Python modules that must run within the virtual environment.
There are multiple ways to run ROS nodes depending on context.

#### Launch Files

If this repository is inside a ROS 2 workspace, use the following commands at
the top level of the workspace.

```shell
source install/setup.$(basename $SHELL)

# Replace LAUNCH_FILE_NAME with one of the files in launch/
ros2 launch yolov8_ros LAUNCH_FILE_NAME
```

#### Direct Invocation

Running nodes directly with Python may be useful for development and testing.
Use the following commands from the top level of this repository.

```shell
# Activate the virtual environment
poetry shell

# Run the node as a Python module, e.g.,
python3 -m yolov8_ros.right_wrist_node
```
