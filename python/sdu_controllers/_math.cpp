// Python bindings for sdu_controllers math utilities and dynamics helpers
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <sdu_controllers/math/forward_dynamics.hpp>
#include <sdu_controllers/math/inverse_dynamics.hpp>
#include <sdu_controllers/math/inverse_dynamics_cartesian_space.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/math/math.hpp>
#include <sdu_controllers/math/pose.hpp>
#include <sdu_controllers/math/pseudoinverse.hpp>
#include <sdu_controllers/math/rnea.hpp>
#include <sdu_controllers/models/robot_model.hpp>

namespace nb = nanobind;

namespace sdu_controllers
{
  nb::module_ create_math_module(nb::module_ &main_module)
  {
    nb::module_ m = main_module.def_submodule("math", "Submodule containing math utilities.");
    m.doc() = "Python bindings for sdu_controllers math utilities.";

    // Ensure core module (and RobotModel bindings) are available for type conversions
    nb::module_::import_("sdu_controllers._sdu_controllers");

    // Pose utilities
    nb::class_<math::Pose>(m, "Pose")
        .def(nb::init<>(), "Default pose (zero position, identity rotation).")
        .def(
            nb::init<const Eigen::Vector3d &, const Eigen::Quaterniond &>(),
            nb::arg("position"),
            nb::arg("orientation"),
            "Construct from position and quaternion (w, x, y, z).")
        .def(
            nb::init<const Eigen::Vector3d &, const Eigen::Vector3d &>(),
            nb::arg("position"),
            nb::arg("angle_axis"),
            "Construct from position and angle-axis vector.")
        .def(nb::init<const std::vector<double> &>(), nb::arg("pose_vector"), "Construct from 6/7 element vector.")
        .def(nb::init<const Eigen::VectorXd &>(), nb::arg("pose_vector"), "Construct from Eigen vector (6/7 elements).")
        .def(nb::init<const std::array<double, 7> &>(), nb::arg("pose_array"), "Construct from array [x,y,z,qw,qx,qy,qz].")
        .def(nb::init<const Eigen::Affine3d &>(), nb::arg("transform"), "Construct from homogeneous transform.")
        .def("get_position", &math::Pose::get_position, "Get position as Vector3d.")
        .def("set_position", &math::Pose::set_position, nb::arg("position"), "Set position.")
        .def("get_orientation", &math::Pose::get_orientation, "Get orientation as quaternion (w, x, y, z).")
        .def("set_orientation", &math::Pose::set_orientation, nb::arg("orientation"), "Set orientation quaternion.")
        .def(
            "to_euler_angles",
            &math::Pose::to_euler_angles,
            nb::arg("order") = std::string("ZYZ"),
            "Return orientation as Euler angles (default ZYZ).")
        .def(
            "to_angle_axis_vector",
            &math::Pose::to_angle_axis_vector,
            "Return orientation as angle-axis vector (axis * angle).")
        .def("to_vector7d", &math::Pose::to_vector7d, "Return [x, y, z, qw, qx, qy, qz].")
        .def("to_std_vector", &math::Pose::to_std_vector, "Return pose as std::vector of length 7.")
        .def("to_transform", &math::Pose::to_transform, "Return pose as homogeneous transform Affine3d.");

    // Core dynamics helpers
    nb::class_<math::InverseDynamics>(m, "InverseDynamics")
        .def(nb::init<std::shared_ptr<models::RobotModel>>(), nb::arg("robot_model"))
        .def("get_robot_model", &math::InverseDynamics::get_robot_model, "Return underlying robot model.");

    nb::class_<math::InverseDynamicsJointSpace, math::InverseDynamics>(m, "InverseDynamicsJointSpace")
        .def(nb::init<std::shared_ptr<models::RobotModel>>(), nb::arg("robot_model"))
        .def(
            "inverse_dynamics",
            &math::InverseDynamicsJointSpace::inverse_dynamics,
            nb::arg("y"),
            nb::arg("q"),
            nb::arg("dq"));

    nb::class_<math::InverseDynamicsCartesianSpace, math::InverseDynamics>(m, "InverseDynamicsCartesianSpace")
        .def(nb::init<std::shared_ptr<models::RobotModel>>(), nb::arg("robot_model"))
        .def(
            "inverse_dynamics",
            &math::InverseDynamicsCartesianSpace::inverse_dynamics,
            nb::arg("y"),
            nb::arg("x_ee"),
            nb::arg("dx_ee"),
            nb::arg("ddx_ee"));

    nb::class_<math::ForwardDynamics>(m, "ForwardDynamics")
        .def(nb::init<std::shared_ptr<models::RobotModel>>(), nb::arg("robot_model"))
        .def("forward_dynamics", &math::ForwardDynamics::forward_dynamics, nb::arg("q"), nb::arg("dq"), nb::arg("tau"));

    // Recursive Newton-Euler Algorithm
    nb::class_<math::RecursiveNewtonEuler>(m, "RecursiveNewtonEuler")
        .def(
            nb::init<models::RobotModel &>(),
            "Initialize the RecursiveNewtonEuler algorithm with a robot model",
            nb::arg("robot_model"))
        .def(
            "inverse_dynamics",
            &math::RecursiveNewtonEuler::inverse_dynamics,
            "Compute inverse dynamics: τ = H(q)q̈ + C(q,q̇)q̇ + g(q)",
            nb::arg("q"),
            nb::arg("dq"),
            nb::arg("ddq"),
            nb::arg("he"))
        .def(
            "forward_dynamics",
            &math::RecursiveNewtonEuler::forward_dynamics,
            "Compute forward dynamics: q̈ = H(q)⁻¹(τ - C(q,q̇)q̇ - g(q))",
            nb::arg("q"),
            nb::arg("dq"),
            nb::arg("tau"))
        .def(
            "inertia",
            &math::RecursiveNewtonEuler::inertia,
            "Compute the joint-space inertia matrix H(q)",
            nb::arg("q"))
        .def(
            "velocity_product",
            &math::RecursiveNewtonEuler::velocity_product,
            "Compute the velocity product term C(q,q̇)q̇ from the manipulator equation",
            nb::arg("q"),
            nb::arg("dq"))
        .def(
            "gravity",
            &math::RecursiveNewtonEuler::gravity,
            "Compute the gravity forces g(q)",
            nb::arg("q"))
        .def(
            "set_z0",
            &math::RecursiveNewtonEuler::set_z0,
            "Set the z-axis of the base frame",
            nb::arg("z0"));

    // Linear algebra utilities
    m.def(
        "rot_vel_transform",
        &math::rot_vel_transform,
        nb::arg("gamma"),
        nb::arg("inverse") = true,
        "Rotation-rate transformation matrix (or its inverse) for ZYZ Euler angles.");

    m.def(
        "jacobian_analytical",
        &math::jacobian_analytical,
        nb::arg("q"),
        nb::arg("robot_model"),
        "Analytical Jacobian using current orientation parametrization (ZYZ Euler).");

    m.def(
        "jacobian_dot_analytical",
        &math::jacobian_dot_analytical,
        nb::arg("q"),
        nb::arg("dq"),
        nb::arg("robot_model"),
        "Time derivative of the analytical Jacobian.");

    m.def(
        "skew",
        [](const Eigen::Vector3d &vec) { return math::skew(vec); },
        nb::arg("vec"),
        "Skew-symmetric matrix from 3D vector.");

    m.def("adjoint", &math::adjoint, nb::arg("transform"), "Adjoint transformation matrix for an SE(3) transform.");

    m.def(
        "wrench_trans",
        &math::wrench_trans,
        nb::arg("torques"),
        nb::arg("forces"),
        nb::arg("transform"),
        "Transform wrench (torques, forces) between frames using adjoint.");

    m.def("quat_exp", &math::exp, nb::arg("quat"), "Quaternion exponential map.");

    m.def(
        "pseudoinverse",
        [](const Eigen::MatrixXd &mat, double tolerance) { return math::pseudoinverse(mat, tolerance); },
        nb::arg("mat"),
        nb::arg("tolerance") = 1e-4,
        "Moore-Penrose pseudoinverse using SVD with tolerance cutoff.");

    return m;
  }
}  // namespace sdu_controllers
