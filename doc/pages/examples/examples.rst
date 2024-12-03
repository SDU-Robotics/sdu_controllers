
.. _examples:

********
Examples
********

This section contains examples which demonstrate the usage of the different controllers:

Admittance controller:

* :ref:`General example<admittance-controller-example>`
* :ref:`Circle example (UR)<admittance-circle-ur-example>`
* :ref:`Kinesthetic teaching example (UR) <kinesthetic-teaching-ur-example>`



Admittance Controller
=====================

.. _admittance-controller-example:

General example
---------------

This example demonstrates the general usage of the admittance controller and shows how it reacts to a force disturbance.
The reference trajectory is a single circle in the xy plane starting at (0.4, 0.3) and moves counter clockwise.
The controller is following this trajectory, when at (0.3, 0.4) a force pointing in positive y is injected.



.. _admittance-circle-ur-example:

Circle example (UR)
-------------------
Desired trajectory: circle
Gain regulates how quickly / forceful robot returns to desired trajectory
Damping is typically set to be critically damped (enough damping to ensure controller stability, 2 * sqrt(k*m))

Important: Set payload on robot!


.. _kinesthetic-teaching-ur-example:

Kinesthetic teaching example (UR)
---------------------------------
Desired trajectory: current position
Gain = 0 (the user can freely move the end effector to any position)
Damping ensures correct sensitivity to forces