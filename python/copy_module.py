#!/usr/bin/env python3
"""
Copy the nanobind-generated versioned module to a standard name.

nanobind automatically adds Python version and platform suffixes to the compiled module
(e.g., _sdu_controllers.cp314-win32.pyd on Windows, _sdu_controllers.cpython-314-x86_64-linux-gnu.so on Linux).
This script copies the generated module to a standard name that can be imported consistently.
"""

import sys
import shutil
import pathlib
import os


def main():
    if len(sys.argv) < 2:
        print("Usage: copy_module.py <target_directory>")
        sys.exit(1)

    target_dir = pathlib.Path(sys.argv[1])
    platform_suffix = ".pyd" if os.name == "nt" else ".so"

    # Find the versioned module file
    versioned_modules = list(
        target_dir.glob("*_sdu_controllers*" + platform_suffix)
    )

    if not versioned_modules:
        print(
            f"Warning: No versioned _sdu_controllers module found in {target_dir}"
        )
        return 1

    src = versioned_modules[0]
    dst = target_dir / f"_sdu_controllers{platform_suffix}"

    if src == dst:
        print(f"Module already has standard name: {src.name}")
        return 0

    try:
        shutil.copy2(str(src), str(dst))
        print(f"Copied module: {src.name} -> {dst.name}")
        return 0
    except Exception as e:
        print(f"Error copying module: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
