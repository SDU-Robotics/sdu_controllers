.. _explanation-impedance:

*******************
Impedance control
*******************

Overview
========

The :ref:`ImpedanceController <impedance-controller-api>` implements operational-space
impedance control as described on page 374 of :cite:t:`2009:Siciliano`. Rather than tracking
a pure motion reference, it regulates the dynamic relationship (the "mechanical impedance")
between motion error and contact wrench :math:`\mathbf{h}^{d}_{e}` at the end-effector:

.. math::

   \mathbf{y} = \mathbf{J}^{-1}\mathbf{M}^{-1}_{d}
     \left(
       \mathbf{K}_{D}\,\dot{\tilde{x}} + \mathbf{K}_{P}\,\tilde{x}
       - \mathbf{M}_{d}\,\dot{\mathbf{J}}\dot{\mathbf{q}}
       + \mathbf{M}_{d}\,\ddot{x}_{d}
       - \mathbf{h}^{d}_{e}
     \right)

The desired inertia :math:`\mathbf{M}_{d}`, stiffness :math:`\mathbf{K}_{P}` and damping
:math:`\mathbf{K}_{D}` shape how the end-effector yields to external forces, instead of
rigidly resisting them. This is what makes impedance control (and its close relative,
:ref:`admittance control <admittance_controller>`) suitable for physical interaction tasks,
whereas :ref:`OSC <explanation-osc>` is purely a motion-tracking controller.

Quaternion orientation error and the :math:`K_{o}'` correction
==================================================================
In :ref:`QUATERNION mode <orientation-representation-api>`, the geometric Jacobian relates
joint velocities to angular velocity :math:`\boldsymbol{\omega}`, not to the time-derivative
of the quaternion vector-part error :math:`\tilde{\boldsymbol{\epsilon}}`. Multiplying
:math:`\mathbf{K}_{P}` directly by :math:`\tilde{\boldsymbol{\epsilon}}` would therefore be
dimensionally inconsistent with the rest of the control law. Following
:cite:t:`1999:Caccavale`, the orientation block of the stiffness matrix is corrected before
use:

.. math::

   \mathbf{K}_{o}' = 2\,\mathbf{E}(\tilde{\boldsymbol{\epsilon}})^{T}\,\mathbf{K}_{P,o},
   \qquad
   \mathbf{E}(\tilde{\boldsymbol{\epsilon}}) = \tilde{\eta}\mathbf{I} - S(\tilde{\boldsymbol{\epsilon}})

where :math:`\tilde{\eta}` is the scalar part of the quaternion error and :math:`S(\cdot)`
is the skew-symmetric operator. This correction is applied internally by
:code:`ImpedanceController::step()` - callers only ever supply the raw desired quaternion
:code:`quat_d`.

Impedance vs. admittance vs. OSC
====================================
A quick way to choose between the three motion controllers in sdu_controllers:

* **OSC** - pure Cartesian motion tracking, no force channel. Use when the end-effector is
  not expected to make contact.
* **Impedance control** - motion tracking *and* a desired contact wrench, computed from an
  inverse-dynamics (torque/acceleration) model. Use when you have an accurate dynamic model
  and want the controller to shape stiffness/damping/inertia directly.
* **Admittance control** - measures external force/torque and adjusts a position/velocity
  reference in response, without requiring an inverse-dynamics model. Use for kinesthetic
  teaching or when running on a robot's built-in position/velocity interface rather than a
  torque interface.

.. seealso::
   :ref:`Impedance control tutorial <impedance_control>` for a worked QUATERNION-mode
   example, and :ref:`Operational-space control <explanation-osc>` for the closely related
   motion-only controller.
