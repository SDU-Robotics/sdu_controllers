from skbuild import setup
import os
import sys
import subprocess


def get_nanobind_dir():
    """Try to get nanobind directory from Python package."""
    try:
        import nanobind
        return os.path.dirname(nanobind.__file__)
    except ImportError:
        pass
    
    # Try using python -m nanobind --cmake_dir
    try:
        result = subprocess.run(
            [sys.executable, "-m", "nanobind", "--cmake_dir"],
            capture_output=True,
            text=True,
            check=False
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except Exception:
        pass
    
    return None


# Set NANOBIND_DIR environment variable if found
nanobind_dir = get_nanobind_dir()
if nanobind_dir:
    os.environ["NANOBIND_DIR"] = nanobind_dir
    print(f"Setting NANOBIND_DIR to: {nanobind_dir}")


setup(
    packages=["sdu_controllers"],
    package_dir={"": "python"},
    zip_safe=False,
    cmake_args=[
        "-DBUILD_TESTING=OFF",
        "-DBUILD_PYTHON=ON",
        "-DBUILD_PYTHON_STUBS=ON",
        "-DBUILD_FOR_PIP_INSTALL=ON",
        "-DBUILD_DOCS=OFF",
        "-DCMAKE_BUILD_TYPE=Release",
    ],
    cmake_install_dir="python/sdu_controllers",
)
