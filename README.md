# YOLOv8 ROS Integration

This repository contains a [Poetry][poetry-home] virtual environment
to use Ultralytics YOLOv8 with ROS 1 Noetic.

[poetry-home]: https://python-poetry.org/

## Getting Started

### Install Python 3.10 (Ubuntu 20.04 only)

The [Deadsnakes Ubuntu PPA][deadsnakes] provides newer Python versions that are
not present in the official package repositories. These commands will set up the
PPA and install the required Python 3.10 packages:

```shell
apt update && apt install software-properties-common
add-apt-repository ppa:deadsnakes/ppa
apt update
apt install python3.10 python3.10-distutils python3.10-venv
```

[deadsnakes]: <https://launchpad.net/~deadsnakes/+archive/ubuntu/ppa>

### Install dependencies to a virtual environment

This repository is set up to build using `catkin` and/or `cmake`. The build
process installs the Poetry dependency manager if it is not already present on
the system and installs Python dependencies to `.venv`.

If this package is within a `catkin` workspace, run `catkin build` within the
workspace. Otherwise, you can run `cmake` directly from the top-level directory
of this repository:

```shell
cmake -B build && cmake --build build
```

### (Optional) Run YOLO Checks

After the virtual environment is initialized you can activate it by running

```shell
poetry shell
```

Then, run YOLO status checks using `yolo check`. You should see output similar
to below:

```shell
$ yolo check
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

## Running ROS Nodes

There are multiple ways to run ROS nodes depending on context.

### Launch Files

If you are in a catkin workspace, source `devel/setup.$(basename $SHELL)` in the
top level of the workspace and then run `roslaunch yolov8-ros $LAUNCH_FILE_NAME`,
where `$LAUNCH_FILE_NAME` is the name of one of the files in
[`launch/`](/launch/).

### Direct Invocation

Using the virtual environment directly is sometimes easier for prototyping and
testing. From the top level of this repository,

```shell
# Activate the virtual environment
poetry shell

# Run the node as a Python module, e.g.,
python3 -m yolov8_ros.right_wrist_node
```
