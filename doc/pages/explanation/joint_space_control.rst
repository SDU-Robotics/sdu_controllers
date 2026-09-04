.. _explanation-joint-space:

*******************************************
Joint-space (inverse dynamics) control
*******************************************

Overview
========

The joint-space motion controller implements inverse dynamics control as described on
page 330 in :cite:t:`2009:Siciliano`. It lets a robot track an arbitrary joint-space
reference trajectory :math:`(q_d, \dot{q}_d, \ddot{q}_d)` within the robot's limits.

The dynamic model of an n-joint robot manipulator given by the Euler-Lagrange equation
can be written as:

.. math::

   B(q)\ddot{q} + C(q, \dot{q})\dot{q} + F\dot{q} + g(q) = u

where :math:`B(q)` is the inertia tensor, :math:`C(q, \dot{q})` is a matrix containing
Coriolis and centrifugal terms, :math:`g(q)` is the gravity vector, and :math:`u` is the
actuator torque. By taking the control :math:`u` as a function of the manipulator state
in the form:

.. math::

   u = B(q)y + C(q, \dot{q})\dot{q} + F\dot{q} + g(q)

the system reduces to :math:`\ddot{q} = y`, where :math:`y` is an auxiliary input signal
that is free to design.

PD control with feedforward
============================
The controller chooses :math:`y` as a PD-control acting on the joint position and
velocity errors, plus a feedforward term:

.. math::

   y = u_{ff} + K_{P}(q_{d} - q) + K_{D}(\dot{q}_{d} - \dot{q})

Typically the desired acceleration is used as feedforward, :math:`u_{ff} = \ddot{q}_d`.

.. note::
    You can also choose the gravity vector as feed-forward, :math:`u_{ff} = g(q)`, which
    makes the controller compensate the gravity forces instead of tracking the reference
    acceleration directly.

Validating the output with forward dynamics
=============================================
To check whether the output torques :math:`u` produce a robot movement that resembles
the input trajectory, a simulation can be performed by using forward dynamics to
calculate an output trajectory from the output torques. This output trajectory can then
be compared with the reference trajectory. The forward dynamics is given by Eq. (7.115),
from page 293 in :cite:t:`2009:Siciliano`:

.. math::

  \ddot{q} = \mathbf{B}^{-1}(q) \left(\tau - \mathbf{C}(q)\dot{q} -\mathbf{\tau}_{g}\right)

which gives the acceleration :math:`\ddot{q}`. This is integrated once to yield the
velocity :math:`\dot{q}`, and integrated again to yield the position :math:`q`.

.. seealso::
   :ref:`Joint-space motion control <joint_space_motion_control>` for a worked tutorial.
