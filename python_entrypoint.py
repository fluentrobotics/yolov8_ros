#! /usr/bin/env python3

import argparse
from types import ModuleType

import yolov8_ros


if __name__ == "__main__":
    valid_modules = []
    for k, v in yolov8_ros.__dict__.items():
        if isinstance(v, ModuleType) and hasattr(v, "main"):
            valid_modules.append(k)

    argparser = argparse.ArgumentParser(prefix_chars="_")
    # roslaunch appends an argument "__name:=name_of_node_in_launch_file"
    argparser.add_argument(
        "__name:",
        type=str,
        choices=valid_modules,
        required=True,
        help="Module to run",
    )

    namespace, _ = argparser.parse_known_args()
    module_name: str = namespace.__getattribute__("name:")

    yolov8_ros.__getattribute__(module_name).main()
