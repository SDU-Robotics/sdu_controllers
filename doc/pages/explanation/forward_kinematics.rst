.. _forward_kinematics_explanation:

*******************
Forward Kinematics
*******************

Overview
========

The forward kinematics module in sdu_controllers provides tools for computing the spatial position
and orientation of robot links given joint configurations. The implementation is designed with
extensibility in mind, using an abstract base class that can support multiple kinematic representations.

Architecture
============

The forward kinematics functionality is built around two main classes:

ForwardKinematics (Base Class)
-------------------------------

The ``ForwardKinematics`` class is an abstract base class that defines the interface for all
kinematic solvers. It provides:

* **Joint Type Support**: Both revolute and prismatic joints are supported through the ``JointType`` enum.
* **Transformation Computation**: Methods to compute homogeneous transformation matrices from the base
  frame to the end-effector or to all intermediate joint frames.
* **Jacobian Computation**: Built-in geometric Jacobian calculation for velocity kinematics and
  differential motion analysis.
* **Flexible Input**: Accepts joint configurations as either ``Eigen::VectorXd`` or ``std::vector<double>``.

Key methods include:

* ``forward_kinematics(q)``: Returns the 4x4 homogeneous transformation matrix to the end-effector.
* ``forward_kinematics_all(q)``: Returns transformation matrices to all joint frames, useful for
  computing Jacobians or analyzing intermediate link positions.
* ``geometric_jacobian(q)``: Computes the 6×DOF geometric Jacobian matrix relating joint velocities
  to end-effector twist.

DHKinematics (DH Parameters)
-----------------------------

The ``DHKinematics`` class is a concrete implementation using the standard Denavit-Hartenberg (DH)
convention. This is one of the most widely used methods for describing robot kinematics.

The DH parameters for each link are:

* **a** (link length): Distance along x-axis from z_{i-1} to z_i
* **alpha** (link twist): Angle about x-axis from z_{i-1} to z_i
* **d** (link offset): Distance along z_{i-1} from x_{i-1} to x_i
* **theta** (joint angle): Angle about z_{i-1} from x_{i-1} to x_i

For revolute joints, theta is the variable; for prismatic joints, d is the variable.

The ``DHKinematics`` constructor accepts DH parameters in several formats:

.. code-block:: cpp

   // Using DHParam structs
   std::vector<DHParam> params = {...};
   DHKinematics fk(params);

   // Using separate vectors
   DHKinematics fk(a, alpha, d, theta, is_joint_revolute);

Example Usage
=============

Here's a simple example of using the DH kinematics:

.. code-block:: cpp

   #include <sdu_controllers/kinematics/dh_kinematics.hpp>

   // Define DH parameters for a 2-DOF robot
   std::vector<double> a = {0.0, 0.5};
   std::vector<double> alpha = {M_PI/2, 0.0};
   std::vector<double> d = {0.1, 0.0};
   std::vector<double> theta = {0.0, 0.0};
   std::vector<bool> is_revolute = {true, true};

   // Create kinematics solver
   sdu_controllers::kinematics::DHKinematics fk(a, alpha, d, theta, is_revolute);

   // Compute forward kinematics
   Eigen::VectorXd q(2);
   q << 0.5, 1.0;  // Joint angles in radians

   Eigen::Matrix4d T = fk.forward_kinematics(q);
   std::cout << "End-effector transform:\n" << T << std::endl;

   // Compute Jacobian
   auto J = fk.geometric_jacobian(q);
   std::cout << "Geometric Jacobian:\n" << J << std::endl;

Future Extensions
=================

The current implementation focuses on serial chain kinematics using standard DH parameters.
Planned future extensions include:

* **Frame Tree Support**: Representation of serial robots using the transform tree to build up the serial links.

* **Alternative DH Conventions**: Support for modified DH parameters.

These extensions will maintain backward compatibility with the existing ``ForwardKinematics`` interface,
allowing users to easily switch between different kinematic representations.
