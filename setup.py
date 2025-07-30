from skbuild import setup


setup(
    packages=["sdu_controllers"],
    package_dir={"": "python"},
    zip_safe=False,
    cmake_args=[
        "-DBUILD_TESTING=OFF",
        "-DBUILD_DOCS=OFF",
        "-DCMAKE_BUILD_TYPE=Release"
    ],
    cmake_install_dir="python/sdu_controllers",
)
