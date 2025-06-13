
.. _installation:

************
Installation
************

There are several ways to use sdu_controllers, if you want to try the C++ examples simply
follow the steps outlined in the :ref:`Building sdu_controllers <building_sdu_controllers>` section.

.. note::
    It is not necessary to install the library to be able to run the examples, this is only relevant
    if you want to use sdu_controllers as a library that you link to your own C++ application.

If you are interested in using the Python module you can follow the instructions given in the
:ref:`Quick install <quick_install>` section.

.. _quick_install:

Quick install
=============
The easiest way if you want to try out sdu_controllers, is to install the python module from PyPI.

.. code-block:: bash

   pip install sdu_controllers


.. _building_sdu_controllers:

Building sdu_controllers
========================

Prerequisites
-------------

Building sdu_controllers requires the following software installed:

* A C++17-compliant compiler
* CMake `>= 3.9`
* Eigen3 `>= 3.3` for linear algebra.
* Doxygen (optional, documentation building is skipped if missing)
* Python `>= 3.8` for building Python bindings

On debian-based linux distributions like Ubuntu, you can install the
dependencies with:

.. code-block:: bash

   sudo apt install build-essential cmake python3-dev python3-pip libeigen3-dev

The following sequence of commands builds sdu_controllers.
It assumes that your current working directory is the top-level directory
of the freshly cloned repository:

.. code-block:: bash

   git submodule update --init --recursive
   mkdir build
   cd build
   cmake -DCMAKE_BUILD_TYPE=Release ..
   cmake --build .

The build process can be customized with the following CMake variables,
which can be set by adding `-D<var>={ON, OFF}` to the `cmake` call:

* `BUILD_TESTING`: Enable building of the test suite (default: `ON`)
* `BUILD_DOCS`: Enable building the documentation (default: `ON`)
* `BUILD_PYTHON`: Enable building the Python bindings (default: `ON`)
* `BUILD_EXAMPLES`: Enable building the examples (default: `ON`)

If you wish to build and install the project as a Python project without
having access to C++ build artifacts like libraries and executables, you
can do so using `pip` from the root directory:

.. code-block:: bash

   git submodule update --init --recursive
   pip install .


Building hardware specific examples
===================================

To avoid any robot specific dependencies in the library and its examples, any examples targeted to a
specific robot have been put in a separate folder which is not built by default. They can be found in the hardware_examples folder.

For building the UR examples ur_rtde is required.

The C++ examples can be build with a CMake option

.. code-block:: bash

    cd build
    cmake -DBUILD_UR_EXAMPLES=ON ..
    cmake --build .

For the Python examples, sdu_controllers Python bindings have to be generated at the root directory

.. code-block:: bash

    pip install .

and the Python dependencies of the examples installed with

.. code-block:: bash

    pip install -r hardware_examples/python/ur/requirements.txt
