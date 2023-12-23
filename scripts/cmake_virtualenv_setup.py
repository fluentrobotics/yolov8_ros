#! /usr/bin/env python3

"""
This repository uses Poetry / PEP 517 to specify and install dependencies, which
is not natively supported by catkin. Run this script in CMakeLists.txt to create
a virtual environment and install Python dependencies.
"""

import os
import shutil
import subprocess
import sys
from pathlib import Path


def assert_pip_installed() -> None:
    try:
        import pip  # noqa: F401
    except ModuleNotFoundError:
        print("[FATAL] pip is not installed on the system.", file=sys.stderr)
        sys.exit(1)


def ensure_poetry() -> None:
    assert_pip_installed()

    # Manually append ~/.local/bin to PATH within this script's environment
    # since we may be running in an old shell unaffected by `pipx ensurepath`.
    os.environ["PATH"] = f"{os.environ['PATH']}:{Path.home()}/.local/bin".strip(":")

    if shutil.which("poetry") is not None:
        return

    # This is one set of the recommended installation instructions for poetry on
    # Ubuntu 20.04 as of Dec 2023.
    try:
        subprocess.run(
            ["python3", "-m", "pip", "install", "--user", "pipx"], check=True
        )
        subprocess.run(["python3", "-m", "pipx", "ensurepath"], check=True)
        subprocess.run(["python3", "-m", "pipx", "install", "poetry"], check=True)
    except subprocess.CalledProcessError as err:
        print(f"[FATAL]: Failed to execute '{err.cmd}'")
        sys.exit(err.returncode)

    if shutil.which("poetry") is None:
        print(
            "[FATAL] Failed to find poetry executable after installation.",
            file=sys.stderr,
        )
        sys.exit(1)


def poetry_create_venv() -> None:
    ensure_poetry()

    try:
        subprocess.run(
            ["poetry", "install", "--no-root", "--no-ansi", "--no-interaction"],
            check=True,
        )
    except subprocess.CalledProcessError as err:
        print("[FATAL]: Failed to create virtualenv.", file=sys.stderr)
        sys.exit(err.returncode)


if __name__ == "__main__":
    poetry_create_venv()
