sdu_controllers
===============
|License badge| |Build badge| |Docs badge|

.. |License badge| image:: https://img.shields.io/badge/License-MIT-yellow.svg
   :target: https://opensource.org/licenses/MIT

.. |Build badge| image:: https://img.shields.io/github/actions/workflow/status/SDU-Robotics/sdu_controllers/ci.yml?branch=main
   :target: https://github.com/SDU-Robotics/sdu_controllers/actions/workflows/ci.yml

.. |Docs badge| image:: https://readthedocs.org/projects/sdu_controllers/badge/
   :target: https://sdu-controllers.readthedocs.io/


.. figure:: _static/sdu_controllers-simulink.svg
   :width: 100%
   :class: only-light

.. figure:: _static/sdu_controllers-simulink-black.svg
   :width: 100%
   :class: only-dark

sdu_controllers is a C++ library that implements fundamental robot controllers. The library is
developed and maintained by the `SDU Robotics
<https://www.sdu.dk/en/forskning/sdurobotics>`_ group at University of Southern Denmark (SDU).
Python bindings and a MATLAB Simulink interface is provided making it possible to use it as a Python library or
in MATLAB Simulink with very little overhead.

---------

In this documentation
---------------------

..  grid:: 1 1 2 2

   ..  grid-item:: :doc:`Tutorial <pages/tutorial/getting_started>`

       **Start here**: a hands-on introduction to sdu_controllers for new users

   ..  grid-item:: :doc:`How-to guides <pages/how_to_guides/installation>`

      **Step-by-step guides** covering key operations and common tasks

.. grid:: 1 1 2 2
   :reverse:

   .. grid-item:: :doc:`Reference <pages/reference/api>`

      **Technical information** - specifications, APIs, architecture

   .. grid-item:: :doc:`Explanation <pages/explanation/index>`

      **Discussion and clarification** of key topics

---------


.. toctree::
   :maxdepth: 2
   :caption: Table of Contents
   :titlesonly:

   pages/tutorial/getting_started
   pages/how_to_guides/installation
   pages/reference/api

