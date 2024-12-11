
.. _installation:

************
Installation
************



Building robot specific examples
--------------------------------

To avoid any robot specific dependencies in the library and its examples, any examples targeted to a
specific robot have been put in a separate folder which is not built by default.

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

    pip install -r examples/python/ur_examples/requirements.txt
