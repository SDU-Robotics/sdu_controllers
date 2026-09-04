.. _how-to-visualize-viser:

*******************************************
Visualize a trajectory with Viser
*******************************************

`Viser <https://viser.studio/>`_ is a browser-based 3D visualization library for Python. It
is used purely for creating short videos of what a controller's motion looks like on a real
robot - it is **not** a dependency of sdu_controllers itself, and the visualization scripts
live separately under :code:`scripts/visualization/`.

.. note::
   sdu_controllers only stores DH parameters for its robot models (no URDF/meshes), so the
   visualization scripts pull a ready-made UR5e description from the
   `robot_descriptions <https://github.com/robot-descriptions/robot_descriptions.py>`_
   package rather than vendoring robot assets in this repository.

Setup
=====

.. code-block:: bash

   pip install -r scripts/visualization/requirements.txt

Playing back a simulation log
================================
Any of the UR5e simulation examples (:ref:`joint-space <joint_space_motion_control>`,
:ref:`Cartesian/OSC <cartesian_motion_control>`, :ref:`impedance <impedance_control>`) log
the joint positions :math:`q_0 \ldots q_5` to a CSV file every control step. Point the
playback script at that file and it will animate a UR5e URDF in the browser:

.. code-block:: bash

   python scripts/visualization/viser_ur5e_playback.py output_impedance.csv \
       --q-columns 1 2 3 4 5 6 --fps 60 --control-rate 500

:code:`--q-columns` selects which 0-indexed CSV columns hold :math:`q_0 \ldots q_5` (this
varies per example - check the header/column order written by the example you ran). The
script also accepts :code:`--fps` to set the browser playback rate and :code:`--control-rate`
to match the sampling frequency used when the CSV was logged.

Open the URL Viser prints in a browser, press :code:`Start playback`, and then use any
screen-recording tool to capture the animation into a video. After the trajectory finishes,
the robot resets to its initial pose and waits for the next click, so you can replay the
same motion as many times as needed while adjusting the camera.
