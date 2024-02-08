import sys
from pathlib import Path

from loguru import logger


logger.remove(0)
logger.add(
    sys.stdout,
    colorize=True,
    format="<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | <level>{level}</level> | <cyan>{file}:{line}</cyan> - <level>{message}</level>",
)


def get_model_download_dir() -> Path:
    # Recursively ascend the parent directories of this file's path looking for
    # the .venv folder.
    for parent in Path(__file__).parents:
        if (parent / ".venv").exists():
            return parent / ".venv/models"

    # If the .venv folder could not be found, just use the current working
    # directory.
    return Path("models")
