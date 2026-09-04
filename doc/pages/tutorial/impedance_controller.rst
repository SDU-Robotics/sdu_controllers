.. _impedance_control:

*******************
Impedance control
*******************
sdu_controllers implements an impedance controller in operational space, as described on
page 374 in :cite:t:`2009:Siciliano`, via the
:ref:`ImpedanceController <impedance-controller-api>` class.

Impedance control differs functionally from the
:ref:`operational-space controller <cartesian_motion_control>` in that it regulates the
*dynamic relationship* between motion tracking error and contact force/torque, rather than
tracking a pure motion reference. This makes it the natural choice when the end-effector is
expected to make contact with the environment (or a person) while following a trajectory:
instead of fighting a disturbance rigidly, the controller lets the desired inertia
:math:`\mathbf{M}_{d}`, stiffness :math:`\mathbf{K}_{P}` and damping :math:`\mathbf{K}_{D}`
shape how it yields to external forces :math:`\mathbf{h}^{d}_{e}`.

Much of the underlying math is shared with the operational-space controller (the same
damped-least-squares Jacobian handling, the same two
:ref:`orientation representations <orientation-representation-api>`), but the control law
and its use cases are distinct - see the :ref:`Explanation <explanation-impedance>` page for
the full derivation.

Orientation representation
===========================
Just like the operational-space controller, :code:`ImpedanceController` supports:

* **ZYZ** (default) - analytical Jacobian + ZYZ Euler-angle orientation error.
* **QUATERNION** - geometric Jacobian + unit-quaternion orientation error (singularity-free).

In QUATERNION mode the orientation part of the stiffness matrix is corrected by the
:math:`\mathbf{K}_{o}'` term (see :cite:t:`1999:Caccavale`) before being applied - this is
handled internally, you only need to supply :code:`quat_d` to :code:`step()`.

Example: compliant circular tracking with orientation excitation (UR5e)
==========================================================================
This example drives a UR5e along a circle in the XY plane (using the shared
:ref:`circular trajectory helper <how-to-generate-circular-trajectory>`) while also
commanding a small sinusoidal yaw oscillation, in QUATERNION mode, with critically-damped
gains (:math:`K_{D} = 2\sqrt{M_{d}K_{P}}`). The code is available under

:code:`examples/simulation/ur/cpp/impedance_controller.cpp`

or

:code:`examples/simulation/ur/python/impedance_controller.py`.

.. tabs::

   .. code-tab:: c++

        #include <common/trajectory_generation.hpp>
        #include <sdu_controllers/controllers/impedance_controller.hpp>
        #include <sdu_controllers/models/ur_robot_model.hpp>

        auto robot_model = std::make_shared<models::URRobotModel>(models::URRobotModel::RobotType::ur5e);

        const double Kp_pos = 8000.0, Kp_orient = 15.0;
        MatrixXd Kp = MatrixXd::Zero(6, 6);
        Kp.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3) * Kp_pos;
        Kp.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3) * Kp_orient;

        VectorXd Md_pos = VectorXd::Ones(3) * 2.5, Md_rot = VectorXd::Ones(3) * 0.5;
        MatrixXd Md = MatrixXd::Zero(6, 6);
        Md.block<3, 3>(0, 0) = Md_pos.asDiagonal();
        Md.block<3, 3>(3, 3) = Md_rot.asDiagonal();

        // Critical damping: D = 2 * sqrt(M * K)
        MatrixXd Kd = MatrixXd::Zero(6, 6);
        Kd.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3) * (2.0 * std::sqrt(Md_pos[0] * Kp_pos));
        Kd.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3) * (2.0 * std::sqrt(Md_rot[0] * Kp_orient));

        controllers::ImpedanceController impedance_controller(
            Kp, Kd, Md, robot_model,
            controllers::ImpedanceController::OrientationRepresentation::QUATERNION);

        // ... in the control loop, at time t:
        Vector3d pos_d, dpos_d, ddpos_d;
        examples::circular_trajectory(center, radius, omega, t, ramp_time, pos_d, dpos_d, ddpos_d);

        Vector4d quat_d; Vector3d omega_d, domega_d;
        examples::yaw_oscillation(R0, axis_base, orient_amp_rad, orient_omega, t, quat_d, omega_d, domega_d);

        VectorXd x_d(6), dx_d(6), ddx_d(6);
        x_d   << pos_d,  Vector3d::Zero();   // orientation commanded via quat_d
        dx_d  << dpos_d, omega_d;
        ddx_d << ddpos_d, domega_d;

        VectorXd h_d_e = VectorXd::Zero(6);  // desired contact wrench (zero: pure motion tracking)
        impedance_controller.step(x_d, dx_d, ddx_d, q, dq, h_d_e, quat_d);
        VectorXd y = impedance_controller.get_output();      // joint accelerations
        VectorXd tau = robot_model->inverse_dynamics(q, dq, y, he);

   .. code-tab:: py

        import sdu_controllers
        from trajectory_generation import circular_trajectory

        robot_model = sdu_controllers.models.URRobotModel(sdu_controllers.models.RobotType.ur5e)

        Kp_pos, Kp_orient = 8000.0, 15.0
        Kp = np.zeros((6, 6))
        Kp[0:3, 0:3] = np.eye(3) * Kp_pos
        Kp[3:6, 3:6] = np.eye(3) * Kp_orient

        Md_pos, Md_orient = 2.5, 0.5
        Md = np.zeros((6, 6))
        Md[0:3, 0:3] = np.eye(3) * Md_pos
        Md[3:6, 3:6] = np.eye(3) * Md_orient

        # Critical damping: D = 2 * sqrt(M * K)
        Kd = np.zeros((6, 6))
        Kd[0:3, 0:3] = np.eye(3) * (2.0 * np.sqrt(Md_pos * Kp_pos))
        Kd[3:6, 3:6] = np.eye(3) * (2.0 * np.sqrt(Md_orient * Kp_orient))

        controller = sdu_controllers.controllers.ImpedanceController(
            Kp, Kd, Md, robot_model,
            sdu_controllers.controllers.OrientationRepresentation.QUATERNION)

        # ... in the control loop, at time t:
        pos_d, dpos_d, ddpos_d = circular_trajectory(center, radius, omega, t, ramp_time)
        # quat_d, omega_d, domega_d from a small yaw oscillation (see the example file)

        x_d   = np.concatenate([pos_d, np.zeros(3)])   # orientation commanded via quat_d
        dx_d  = np.concatenate([dpos_d, omega_d])
        ddx_d = np.concatenate([ddpos_d, domega_d])

        h_d_e = np.zeros(6)   # desired contact wrench (zero: pure motion tracking)
        controller.step(x_d, dx_d, ddx_d, q, dq, h_d_e, quat_d)
        y = controller.get_output()
        tau = robot_model.inverse_dynamics(q, dq, y, he)

.. note::
   Setting :code:`h_d_e` (desired contact wrench) to a non-zero value lets the controller
   actively regulate a target contact force while remaining compliant along the other axes -
   this is what differentiates impedance control from :ref:`admittance control
   <admittance_controller>` (which instead measures external force and modifies its motion
   in response) and from OSC (pure motion tracking, no force channel).

.. seealso::
   :ref:`Visualize a trajectory with Viser <how-to-visualize-viser>` to turn the logged
   output into a short video of the robot motion.
