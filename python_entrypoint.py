#! /usr/bin/env python3

import argparse
from types import ModuleType

import yolov8_ros


if __name__ == "__main__":
    valid_modules = []
    for k, v in yolov8_ros.__dict__.items():
        if isinstance(v, ModuleType) and hasattr(v, "main"):
            valid_modules.append(k)

    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "module", type=str, help="Module to run", choices=valid_modules
    )

    namespace, _ = argparser.parse_known_args()
    module: str = namespace.module

    yolov8_ros.__getattribute__(module).main()
