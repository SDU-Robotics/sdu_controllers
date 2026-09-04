.. _getting-started:

***************
Getting started
***************

To get started with using sdu_controllers, you first need to install it on your system. See
:ref:`Installation <installation>`.

Building blocks
===============
Every example in this tutorial is built from the same three pieces:

* **A robot model** - describes the kinematics and dynamics of the manipulator, and provides
  quantities such as the mass matrix, gravity vector and inverse dynamics. Models are
  configured from the YAML files in :code:`config/models/`, and ready-made models such as
  :code:`URRobotModel` are included.
* **A controller** - turns a desired state and the measured state into a control output.
  sdu_controllers provides joint-space, Cartesian, impedance and admittance controllers.
* **A reference trajectory** - the desired motion you want the robot to follow, supplied by
  you as a sequence of desired positions, velocities and accelerations.

sdu_controllers is written in C++ and exposes the same API through Python bindings, so the
tutorials show both languages side by side.

The control loop
================
Regardless of which controller you use, the structure of the program is the same: at every
timestep you evaluate the reference, step the controller with the measured state, and map
the controller output to joint torques through the robot model.

.. code-block:: text

   for each timestep t:
       q_d, dq_d, ddq_d = reference(t)                     # desired motion
       controller.step(q_d, dq_d, u_ff, q, dq)             # feedback + feedforward
       y   = controller.get_output()
       tau = robot_model.inverse_dynamics(q, dq, y, he)    # joint torques

Once you recognise this loop, the remaining tutorials are variations on it: a different
controller, a different robot model, or a different reference.

A first reference trajectory
============================
A convenient reference to start with is a sinusoid on every joint. It is smooth
(infinitely differentiable), periodic and easy to reason about, which makes tracking errors
easy to spot. The desired joint position, velocity and acceleration are simply

.. math::

   q_d(t) = q_0 + a \sin(\omega t), \quad
   \dot{q}_d(t) = a \omega \cos(\omega t), \quad
   \ddot{q}_d(t) = -a \omega^2 \sin(\omega t)

where :math:`q_0` is a chosen home position, :math:`a` is the amplitude, and
:math:`\omega = 2\pi f` for a chosen frequency :math:`f`. Because :math:`q_d(0) = q_0`,
the robot starts the motion from rest with no initial jump in the reference. Increasing
:math:`a` or :math:`f` makes the tracking test more demanding.

The following video shows this kind of sinusoidal joint motion running on a UR5e manipulator:

.. raw:: html
   :class: only-light

   <div style="width:75%; max-width:720px; cursor:pointer; border-radius:6px; overflow:hidden;"
        title="Click to pause / resume"
        onclick="var v=this.querySelector('video'); v.paused ? v.play() : v.pause();">
     <video autoplay loop muted playsinline
            style="width:100%; display:block; border-radius:6px;">
       <source src="../../_static/videos/ur5e_sinusoidal_trajectory_light.webm" type="video/webm">
     </video>
   </div>

.. raw:: html
   :class: only-dark

   <div style="width:75%; max-width:720px; cursor:pointer; border-radius:6px; overflow:hidden;"
        title="Click to pause / resume"
        onclick="var v=this.querySelector('video'); v.paused ? v.play() : v.pause();">
     <video autoplay loop muted playsinline
            style="width:100%; display:block; border-radius:6px;">
       <source src="../../_static/videos/ur5e_sinusoidal_trajectory_dark.webm" type="video/webm">
     </video>
   </div>


.. tip::
    For trajectories that come from a planner or a recorded file instead, see
    :ref:`Use a custom trajectory <how-to-custom-trajectory>`.

Next steps
==========
You now know the three building blocks and the shape of the control loop. The next page
wires them together on a UR5e manipulator:

* :ref:`Joint-space motion control <joint_space_motion_control>` - **start here**
* :ref:`Cartesian motion control <cartesian_motion_control>`
* :ref:`Impedance control <impedance_control>`
* :ref:`Admittance control <admittance_controller>`

.. seealso::
   :ref:`Joint-space (inverse dynamics) control <explanation-joint-space>` for the theory
   behind the controller used in the next tutorial.
