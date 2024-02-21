#! /usr/bin/env python3

"""
This script is intended to be executed from a ROS 2 XML launch file to run a
Python module as a node within a virtual environment. Virtual environments are
frequently used to avoid versioning conflicts of Python package dependencies,
particularly for learning-based projects.

Recall that Python modules are run using the -m flag, e.g.,
  python3 -m pypackage.module
This is incompatible with ROS 2 XML launch files since the "exec" property of
the <node> tag expects a single executable file. This script functions as an
executable file for the "exec" property.

Importantly, this script must be executed inside the virtual environment, e.g.,
  .venv/bin/python3 python_entrypoint.py
Although this script uses /usr/bin/env in its shebang, PATH should not be relied
upon to provide the correct python3 executable since attempting to find and
source the proper ".venv/bin/activate" from an XML launch file would be
extremely messy. Instead, assuming that this script ("python_entrypoint.py") is
located in the following directory structure after building a ROS 2 workspace:

  ${CMAKE_INSTALL_PATH}/ (typically: ws/install/project_name)
  ├── .venv/
  ├── lib/
  │   └── project_name/
  │       ├── python_entrypoint.py
  │       ├── pypackage1/
  │       │   ├── __init__.py
  │       │   └── module_a.py
  │       └── pypackage2/
  │           ├── __init__.py
  │           └── module_b.py
  └── share/
      └── project_name/
          └── launch/
              └── launch_file.launch

this script can be executed from a launch file using the "launch-prefix"
property of the <node> tag. In the following example, a node is created by
running "module_a" of Python package "pypackage1" within the ROS 2 package
"project_name".

 <node
   pkg="project_name"
   name="module_a"
   launch-prefix="$(find-pkg-prefix project_name)/.venv/bin/python"
   exec="python_entrypoint.py"
   ...
 >

The directory structure is not arbitrary: ROS 2 expects launch files to be
located in "${CMAKE_INSTALL_PATH}/share/" and executables to be located in
"${CMAKE_INSTALL_PATH}/lib/". In the ROS 2 XML launch file specification,
$(find-pkg-prefix project_name) expands to ${CMAKE_INSTALL_PATH}.

The objectives of this script are as follows:
1. Create Python module objects for module_a and module_b dynamically,
   i.e., without knowing the names "pypackage1", "pypackage2", "module_a", or
   "module_b".
2. Read the name of a node from the command invoked by a launch file and run the
   main() function of the corresponding Python module, again without knowing the
   name of the package that the module belongs to.

Assumptions and Limitations:
1. All Python packages and modules must be uniquely named.
2. Python modules functioning as nodes need to have a main() function.
3. This script needs to run each module to determine whether the
   module contains a main() function.
"""

import argparse
import importlib.util
import pkgutil
import sys
from pathlib import Path
from types import FunctionType, ModuleType


def find_valid_modules() -> dict[str, ModuleType]:
    # Valid modules are defined as modules with a main() function.
    valid_modules: dict[str, ModuleType] = {}

    project_dir = Path(__file__).parent

    project_packages_info: list[pkgutil.ModuleInfo] = [
        module
        for module in pkgutil.iter_modules(path=[str(project_dir)])
        if module.ispkg
    ]

    # Find valid modules within this project's packages.
    for package_info in project_packages_info:
        # Importing the package is necessary since submodules may reference the
        # package by importing from it, which requires the package to be loaded
        # and present in sys.modules. This process is somewhat convoluted and
        # filled with landmines:
        # https://docs.python.org/3.10/library/importlib.html#checking-if-a-module-can-be-imported
        package_spec = package_info.module_finder.find_spec(package_info.name)  # type: ignore
        if package_spec is None or package_spec.loader is None:
            continue
        package: ModuleType = importlib.util.module_from_spec(package_spec)
        sys.modules[package_info.name] = package
        package_spec.loader.exec_module(package)

        for submodule_info in pkgutil.iter_modules(
            path=package_spec.submodule_search_locations
        ):
            # Import the submodule to inspect its contents. Note that it does
            # not seem to be necessary to modify sys.modules here.
            submodule_spec = submodule_info.module_finder.find_spec(submodule_info.name)  # type: ignore
            if submodule_spec is None or submodule_spec.loader is None:
                continue
            submodule = importlib.util.module_from_spec(submodule_spec)
            submodule_spec.loader.exec_module(submodule)

            try:
                if isinstance(getattr(submodule, "main"), FunctionType):
                    valid_modules[submodule_info.name] = submodule
            except AttributeError:
                pass

    return valid_modules


if __name__ == "__main__":
    valid_modules = find_valid_modules()

    # Read the module name from command-line arguments. The ROS 2 launch system
    # appends the command-line argument "__node:=name_of_node".
    argparser = argparse.ArgumentParser(prefix_chars="_")
    argparser.add_argument(
        "__node:",
        type=str,
        choices=valid_modules,
        required=True,
        help="Module to run",
    )

    namespace, _ = argparser.parse_known_args()
    module_name: str = namespace.__getattribute__("node:")

    valid_modules[module_name].main()
