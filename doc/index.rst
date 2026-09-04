sdu_controllers
===============
|License badge| |Build badge| |Docs badge|

.. |License badge| image:: https://img.shields.io/badge/License-MIT-yellow.svg
   :target: https://opensource.org/licenses/MIT

.. |Build badge| image:: https://img.shields.io/github/actions/workflow/status/SDU-Robotics/sdu_controllers/ci.yml?branch=main
   :target: https://github.com/SDU-Robotics/sdu_controllers/actions/workflows/ci.yml

.. |Docs badge| image:: https://readthedocs.org/projects/sdu_controllers/badge/
   :target: https://sdu-controllers.readthedocs.io/

.. .. figure:: _static/joint_motion_control.svg
   :width: 90%
   :class: only-light

.. .. figure:: _static/joint_motion_control.svg
   :width: 90%
   :class: only-dark

sdu_controllers is a C++ library that implements fundamental robot controllers. The library is
developed and maintained by the `SDU Robotics
<https://www.sdu.dk/en/forskning/sdurobotics>`_ group at University of Southern Denmark (SDU).
Python bindings and a MATLAB Simulink interface is provided making it possible to use it as a Python library or
in MATLAB Simulink with very little overhead.


In this documentation
---------------------

.. only:: html and not epub

   .. grid:: 2
      :gutter: 3

      .. grid-item-card:: :octicon:`mortar-board;1.2em;sd-text-primary sd-mr-2 sd-align-middle` Tutorials
         :link: pages/tutorial/index
         :link-type: doc
         :class-title: sd-text-primary sd-font-weight-bold

         **Start here:** A hands-on introduction to sdu_controllers for new users.

      .. grid-item-card:: :octicon:`tools;1.2em;sd-text-primary sd-mr-2 sd-align-middle` How-to guides
         :link: pages/how_to_guides/index
         :link-type: doc
         :class-title: sd-text-primary sd-font-weight-bold

         **Step-by-step guides** covering common tasks and key functionalities.

      .. grid-item-card::  :octicon:`light-bulb;1.2em;sd-text-primary sd-mr-2 sd-align-middle` Explanation
         :link: pages/explanation/index
         :link-type: doc
         :class-title: sd-text-primary sd-font-weight-bold

         **Discussion and clarification** of key topics

      .. grid-item-card::  :octicon:`book;1.2em;sd-text-primary sd-mr-2 sd-align-middle` Reference
         :link: pages/reference/index
         :link-type: doc
         :class-title: sd-text-primary sd-font-weight-bold

         **Technical information** - specifications, APIs, architecture

.. only:: epub or latex

   * :doc:`Tutorials <pages/tutorial/index>` - A hands-on introduction to sdu_controllers for new users.
   * :doc:`How-to guides <pages/how_to_guides/index>` - Step-by-step guides covering common tasks and key functionalities.
   * :doc:`Explanation <pages/explanation/index>` - Discussion and clarification of key topics.
   * :doc:`Reference <pages/reference/index>` - Technical information, specifications, APIs, and architecture.

---------

Acknowledgements
----------------

This work has been carried out within the framework of the EUROfusion Consortium, funded by the European Union 
via the Euratom Research and Training Programme (Grant Agreement No 101052200 — EUROfusion). 
Views and opinions expressed are however those of the author(s) only and do not necessarily 
reflect those of the European Union or the European Commission. Neither the European Union nor 
the European Commission can be held responsible for them.

.. toctree::
   :hidden:
   :maxdepth: 3
   :caption: Table of Contents

   pages/tutorial/index
   pages/how_to_guides/index
   pages/reference/index
   pages/explanation/index

