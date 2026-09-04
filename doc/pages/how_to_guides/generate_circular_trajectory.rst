.. _how-to-generate-circular-trajectory:

*******************************************
Generate a circular Cartesian trajectory
*******************************************

All of the Cartesian tutorials (:ref:`cartesian_motion_control`, :ref:`impedance_control`)
and the :ref:`admittance controller <admittance_controller>` examples track a circle in the
XY plane rather than a one-off point-to-point move. This is deliberate: a circle is a
closed, repeatable, continuous-tracking reference that makes it easy to compare how
different controllers behave (steady-state error, response to disturbances, overshoot on
orientation, ...).

Why a common function instead of a file?
==========================================
Unlike an arbitrary offline-planned joint trajectory (see
:ref:`Track a custom/offline recorded trajectory <how-to-custom-trajectory>`), a circle has
a closed-form position, velocity and acceleration at any time :math:`t` - there is no need
to pre-compute it with an external planner and persist it to a CSV file. Instead, every
tutorial and example calls one shared helper function directly inside the control loop:

* C++: :code:`sdu_controllers::math` in :code:`include/sdu_controllers/math/trajectory_generation.hpp`
* Python: :code:`sdu_controllers.common`

Position trajectory
=====================
The desired position on a circle of radius :math:`r` centred at :math:`\mathbf{c}` in the
XY plane is:

.. math::

   \mathbf{p}_{d}(t) = \begin{bmatrix} c_x + r\cos\theta(t) \\ c_y + r\sin\theta(t) \\ c_z \end{bmatrix}

with :math:`\theta(t) = \omega t` giving the corresponding velocity and acceleration by
differentiation. To avoid a torque discontinuity at :math:`t=0` (where a constant
:math:`\omega` would imply an instantaneous jump in velocity), the angular speed is ramped
up from :math:`0` to :math:`\omega` over a short :code:`ramp_time` using a quintic profile:

.. math::

   \omega(t) = \omega \left(10u^3 - 15u^4 + 6u^5\right), \quad u = t / \text{ramp\_time}

This profile is :math:`C^2` continuous (zero acceleration and jerk at the ramp's start and
end), which keeps the commanded acceleration - and therefore the torques - smooth.

Usage
=====

.. tabs::

   .. code-tab:: c++

        #include <sdu_controllers/math/trajectory_generation.hpp>

        Eigen::Vector3d pos_d, vel_d, acc_d;
        sdu_controllers::math::circular_trajectory(center, radius, omega, t, ramp_time, pos_d, vel_d, acc_d);

   .. code-tab:: py

        from sdu_controllers.common import circular_trajectory

        pos_d, vel_d, acc_d = circular_trajectory(center, radius, omega, t, ramp_time)

Orientation excitation (QUATERNION mode)
==========================================
To exercise the orientation channel of a controller running in
:ref:`QUATERNION mode <orientation-representation-api>`, a small sinusoidal yaw oscillation
about a fixed axis can be added on top of the positional circle using
:code:`sdu_controllers::math::yaw_oscillation()` (C++) - this returns a desired quaternion
:code:`[w, x, y, z]` together with the corresponding angular velocity/acceleration, ready to
pass to :code:`ImpedanceController::step()` or :code:`OperationalSpaceController::step()`.
See :ref:`impedance_control` for a complete example.

.. seealso::
   :ref:`Track a custom/offline recorded trajectory <how-to-custom-trajectory>` for the
   file-based waypoint-replay approach used by the joint-space tutorial.
