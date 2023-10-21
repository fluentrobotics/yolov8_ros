# YOLOv8 Virtual Environment

This repository contains a [Poetry][poetry-home] virtual environment
specification to use Ultralytics YOLOv8.

> [!Note]
> Ultralytics provides alternative installation methods including Conda and
> Docker. See <https://github.com/ultralytics/ultralytics>

Table of Contents

[poetry-home]: https://python-poetry.org/

## Getting Started

### Install Python 3.10 (Ubuntu 20.04 only)

#### Option 1: Deadsnakes Ubuntu PPA

The [Deadsnakes Ubuntu PPA][deadsnakes] provides newer Python versions that are
not present in the official package repositories. These commands will set up the
PPA and install the required Python 3.10 packages:

```shell
$ sudo add-apt-repository ppa:deadsnakes/ppa
$ sudo apt update
$ sudo apt install python3.10 python3.10-distutils python3.10-venv
```

[deadsnakes]: <https://launchpad.net/~deadsnakes/+archive/ubuntu/ppa>

#### Option 2: Build from source

Follow Section 2.2 from the [Python documentation][python-build].

[python-build]: <https://docs.python.org/3.10/using/unix.html>

### Install the Poetry Python dependency manager

Follow the instructions [here][poetry-install]. If you want to install Poetry
system-wide, follow the instructions in Section 2 "Install Poetry (advanced)"
to set `POETRY_HOME`.

Afterwards, verify that Poetry is installed and in your `$PATH`.

```shell
$ poetry about
Poetry - Package Management for Python

Version: 1.6.1
Poetry-Core Version: 1.7.0

Poetry is a dependency manager tracking local dependencies of your projects and libraries.
See https://github.com/python-poetry/poetry for more information.
```

[poetry-install]: <https://python-poetry.org/docs/#installation>

#### Recommended post-installation steps

Configure Poetry to put virtual environment directories in the project
directory. Virtual environments will be installed to `$PROJECT_DIR/.venv`.

```shell
poetry config virtualenvs.in-project true
```

### Install the Virtual Environment

> [!Note]
> If you do not intend to use CUDA, change the PyTorch packages in
> [`pyproject.toml`](./pyproject.toml) from CUDA to CPU by swapping the comment
> status of these lines:
>
> ```toml
> # torch = {version = "*", source = "PyTorch CPU"}
> # torchaudio = {version = "*", source = "PyTorch CPU"}
> # torchvision = {version = "*", source = "PyTorch CPU"}
> torch = {version = "*", source = "PyTorch CUDA 11.8"}
> torchaudio = {version = "*", source = "PyTorch CUDA 11.8"}
> torchvision = {version = "*", source = "PyTorch CUDA 11.8"}
> ```
>
> This will save a considerable amount of time and disk space.

Run

```shell
poetry install --no-root
```

### Running Checks

After the virtual environment is installed you can activate it by running

```shell
poetry shell
```

Then, run YOLO status checks using `yolo check`. You should see similar output
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
