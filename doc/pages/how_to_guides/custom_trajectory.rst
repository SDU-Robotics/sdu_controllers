.. _how-to-custom-trajectory:

***************************************************
Track a custom / offline recorded trajectory
***************************************************

Not every trajectory can be described in closed form like the
:ref:`circular trajectory <how-to-generate-circular-trajectory>` used throughout the other
tutorials. If you need to track an arbitrary joint-space motion - e.g. planned offline with
an external tool, or recorded from a real robot - sdu_controllers reads it from a CSV file
of pre-computed waypoints instead.

This is the approach used by the :ref:`joint-space motion control <joint_space_motion_control>`
tutorial's "first steps" example.

Generating a waypoint file
============================
Pick a start and end joint configuration, :math:`q_{start}` and :math:`q_{final}`, then use
the provided script to interpolate between them with a quintic (5th order) polynomial via
the Robotics Toolbox for Python (:cite:t:`2021:Corke`):

.. code-block:: bash

   python3 scripts/trajectory/generate_joint_trajectory.py

This writes :code:`data/joint_trajectory_safe.csv`, with each row containing joint
position, velocity and acceleration (:math:`q, \dot{q}, \ddot{q}`) for every timestep.

Loading it in your controller
================================
:code:`sdu_controllers::utils::get_trajectory_from_file()` (C++) or
:code:`numpy.genfromtxt()` (Python) reads the CSV back into memory; each row is then split
into :math:`q_d, \dot{q}_d, \ddot{q}_d` and fed to the controller's :code:`step()` once per
control-loop iteration - see the :ref:`joint-space tutorial <ur5e_joint_space_control>` for
the full loop.

.. note::
   Prefer the :ref:`circular trajectory helper <how-to-generate-circular-trajectory>` when
   your reference motion *can* be expressed in closed form - it avoids the extra
   generate/read-from-file step entirely.
