***************
Getting started
***************

To get started with using sdu_controllers, you first need to install it on your system. See
:ref:`Installation <installation>`.

First steps
===========
sdu_controllers implements a fundamental joint-space motion controller. By using this controller you can make a
robot follow an arbitrary joint-space trajectory, within the defined limits of the robots. In this example we use
the Universal Robots UR5e 6DOF robot manipulator, see also the example with the 7DOF breeding blanket handling robot
here.

Example with Universal Robots UR5e robot
========================================
First a trajectory must be generated. For the sake of the example we choose two arbitrary joint positions. In
this example the robot is moved from the joint position :math:`q_{start}` defined as:

.. math::

   q_{start} = [0^{\circ}, -90^{\circ}, -90^{\circ}, -90^{\circ}, 90^{\circ}, 0^{\circ}]

to the joint position :math:`q_{final}` defined as:

.. math::

   q_{final} = [45^{\circ}, -120^{\circ}, -90^{\circ}, -60.0^{\circ}, 90^{\circ}, -45^{\circ}]
.. tabs::

   .. code-tab:: c++

         int main(const int argc, const char **argv) {
           return 0;
         }

   .. code-tab:: py

         def main():
             return



Example with Breeding Blanket Handling Robot
============================================

.. tabs::

   .. code-tab:: c++

         int main(const int argc, const char **argv) {
           return 0;
         }

   .. code-tab:: py

         def main():
             return


see additional examples in the :ref:`Examples <examples>` section.