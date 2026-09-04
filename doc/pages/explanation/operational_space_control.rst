.. _explanation-osc:

*******************************************
Operational-space control (OSC)
*******************************************

Overview
========

The :ref:`OperationalSpaceController <operational-space-controller-api>` inverts the
task-space dynamics directly, so that a desired Cartesian pose/velocity/acceleration maps
to joint accelerations without an intermediate inverse-kinematics step. It implements
Eq. (8.114) from page 348 of :cite:t:`2009:Siciliano`:

.. math::

   \mathbf{y} = \mathbf{J}^{-1}(q)\left(\ddot{x}_{d} + \mathbf{K}_{D}\dot{\tilde{x}} +
   \mathbf{K}_{P}\tilde{x} - \mathbf{\dot{J}}(q, \dot{q})\dot{q}\right)

where :math:`\tilde{x} = x_d - x_e` is the Cartesian pose error and :math:`\mathbf{J}` is
either the analytical Jacobian :math:`\mathbf{J}_A` (ZYZ mode) or the geometric Jacobian
(QUATERNION mode) - see :ref:`OrientationRepresentation <orientation-representation-api>`.

Damped least-squares inversion
================================
Because :math:`\mathbf{J}` is not square for redundant manipulators (and can be
ill-conditioned near singularities even for 6-DOF arms), the controller does not invert
:math:`\mathbf{J}` directly. Instead it uses the damped least-squares (DLS) pseudo-inverse:

.. math::

   \mathbf{J}^{+} = \mathbf{J}^{T}\left(\mathbf{J}\mathbf{J}^{T} + \kappa^2 \mathbf{I}\right)^{-1}

controlled by :code:`set_kappa()`. Increasing :math:`\kappa` trades tracking accuracy for
robustness near singular configurations.

Redundancy resolution
========================
For manipulators with more actuated joints than task-space dimensions (e.g. the 7-DOF
breeding blanket handling robot), :math:`\mathbf{J}^{+}` has a non-trivial null space: any
joint velocity in
:math:`\mathcal{N} = \mathbf{I} - \mathbf{J}^{+}\mathbf{J}` (sized to the robot's actuated
DOF, via :code:`RobotModel::get_dof()`) produces zero end-effector motion. The controller
uses this freedom for a secondary joint-damping task that suppresses drift on the redundant
joints, combined with the primary task as:

.. math::

   \mathbf{y} = \mathbf{J}^{+}\left(\ddot{x}_{d} + \mathbf{K}_{D}\dot{\tilde{x}} +
   \mathbf{K}_{P}\tilde{x} - \mathbf{\dot{J}}\dot{q}\right)
   + \mathcal{N}\left(-K_{D,\text{null}}\dot{q}\right)

tunable via :code:`set_Kd_null()`. On a non-redundant 6-DOF arm, :math:`\mathcal{N}` is
(numerically) zero and the secondary task has no effect.

.. seealso::
   :ref:`Cartesian motion control <cartesian_motion_control>` for a worked tutorial, and
   :ref:`Impedance control <explanation-impedance>` for the closely related controller that
   adds a contact-force channel.
