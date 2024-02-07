#! /usr/bin/env python3

# Assuming the following project structure:
#
#   project_dir
#   ├── scripts
#   │    ├── __file__
#   │    └── ...
#   ├── package1
#   │   ├── __init__.py
#   │   └── module_a.py
#   ├── package2
#   │   ├── __init__.py
#   │   └── module_b.py
#   └── ...
#
# The objectives of this script are as follows:
# 1. Create Python module objects for module_a and module_b dynamically,
#    i.e., without knowing the names "package1", "package2", "module_a", or
#    "module_b".
# 2. Read the name of a node spawned by an invocation of this script from a
#    launch file and run the main() method of the respective module.

import argparse
import importlib.util
import pkgutil
import sys
from types import FunctionType, ModuleType
from pathlib import Path


def find_valid_modules() -> dict[str, ModuleType]:
    # Valid modules are modules with a main() function.
    valid_modules: dict[str, ModuleType] = {}

    project_dir = Path(__file__).parent.parent

    project_packages_info = [
        module
        for module in pkgutil.iter_modules(path=[str(project_dir)])
        if module.ispkg
    ]

    # Find valid modules within this project's packages.
    for package_info in project_packages_info:
        package_spec = package_info.module_finder.find_spec(package_info.name, str(project_dir))  # type: ignore
        if package_spec is None or package_spec.loader is None:
            continue

        # Importing the package is somewhat convoluted.
        # https://docs.python.org/3.10/library/importlib.html#checking-if-a-module-can-be-imported
        package = importlib.util.module_from_spec(package_spec)
        sys.modules[package_info.name] = package
        package_spec.loader.exec_module(package)

        # Find modules within this package that have a main() function.
        for k, v in package.__dict__.items():
            try:
                if isinstance(v, ModuleType) and isinstance(
                    getattr(v, "main"), FunctionType
                ):
                    valid_modules[k] = v
            except AttributeError:
                pass

    return valid_modules


if __name__ == "__main__":
    valid_modules = find_valid_modules()

    # Read the module name from command-line arguments. roslaunch appends the
    # command-line argument "__name:=name_property_in_launch_file"
    argparser = argparse.ArgumentParser(prefix_chars="_")
    argparser.add_argument(
        "__name:",
        type=str,
        choices=valid_modules,
        required=True,
        help="Module to run",
    )

    namespace, _ = argparser.parse_known_args()
    module_name: str = namespace.__getattribute__("name:")

    valid_modules[module_name].main()
