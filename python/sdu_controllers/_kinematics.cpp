// Python bindings for sdu_controllers math utilities and dynamics helpers
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <sdu_controllers/kinematics/dh_kinematics.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>

namespace nb = nanobind;

namespace sdu_controllers
{
  nb::module_ create_kinematics_module(nb::module_ &main_module)
  {
    nb::module_ m = main_module.def_submodule("kinematics", "Submodule containing kinematics utilities.");
    m.doc() = "Python bindings for sdu_controllers kinematics utilities.";

    // Bind JointType enum
    nb::enum_<kinematics::ForwardKinematics::JointType>(m, "JointType")
        .value("REVOLUTE", kinematics::ForwardKinematics::JointType::REVOLUTE, "Revolute joint")
        .value("PRISMATIC", kinematics::ForwardKinematics::JointType::PRISMATIC, "Prismatic joint")
        .export_values();

    // Bind DHParam structure
    nb::class_<kinematics::DHParam>(m, "DHParam", "Denavit-Hartenberg parameters for a single link")
        .def(nb::init<>(), "Default constructor")
        .def_rw("a", &kinematics::DHParam::a, "Link length")
        .def_rw("alpha", &kinematics::DHParam::alpha, "Link twist")
        .def_rw("d", &kinematics::DHParam::d, "Link offset")
        .def_rw("theta", &kinematics::DHParam::theta, "Joint angle")
        .def_rw(
            "is_joint_revolute", &kinematics::DHParam::is_joint_revolute, "True if joint is revolute, false if prismatic");

    // Bind ForwardKinematics base class
    nb::class_<kinematics::ForwardKinematics>(m, "ForwardKinematics", "Base class for forward kinematics implementations")
        .def(
            "forward_kinematics",
            &kinematics::ForwardKinematics::forward_kinematics,
            nb::arg("q"),
            "Get the transformation matrix from base to end-effector\n\n"
            "Args:\n"
            "    q: Joint configuration\n\n"
            "Returns:\n"
            "    4x4 homogeneous transformation matrix to end-effector")
        .def(
            "forward_kinematics_all",
            &kinematics::ForwardKinematics::forward_kinematics_all,
            nb::arg("q"),
            "Get the transformation matrices from base to each joint frame\n\n"
            "Args:\n"
            "    q: Joint configuration\n\n"
            "Returns:\n"
            "    List of 4x4 homogeneous transformation matrices to each joint frame")
        .def(
            "get_joint_types",
            &kinematics::ForwardKinematics::get_joint_types,
            nb::rv_policy::reference_internal,
            "Get the type of each joint in the kinematic chain\n\n"
            "Returns:\n"
            "    List of joint types (REVOLUTE or PRISMATIC)")
        .def(
            "geometric_jacobian",
            nb::overload_cast<const Eigen::VectorXd &>(&kinematics::ForwardKinematics::geometric_jacobian, nb::const_),
            nb::arg("q"),
            "Compute the geometric Jacobian at the given joint configuration\n\n"
            "Args:\n"
            "    q: Joint configuration\n\n"
            "Returns:\n"
            "    6xDOF geometric Jacobian matrix")
        .def(
            "geometric_jacobian",
            nb::overload_cast<const std::vector<Eigen::Matrix4d> &>(
                &kinematics::ForwardKinematics::geometric_jacobian, nb::const_),
            nb::arg("fk_matrices"),
            "Compute the geometric Jacobian using precomputed forward kinematics matrices\n\n"
            "Args:\n"
            "    fk_matrices: Precomputed forward kinematics matrices for each joint\n\n"
            "Returns:\n"
            "    6xDOF geometric Jacobian matrix")
        .def(
            "get_dof",
            &kinematics::ForwardKinematics::get_dof,
            "Get the degrees of freedom of the kinematic chain\n\n"
            "Returns:\n"
            "    Number of degrees of freedom");

    // Bind DHKinematics class
    nb::class_<kinematics::DHKinematics, kinematics::ForwardKinematics>(
        m, "DHKinematics", "Forward kinematics using Denavit-Hartenberg parameters")
        .def(nb::init<>(), "Default constructor")
        .def(
            nb::init<const std::vector<kinematics::DHParam> &>(),
            nb::arg("dh_parameters"),
            "Construct a DHKinematics object using DH parameters\n\n"
            "Args:\n"
            "    dh_parameters: List of DH parameters for each link")
        .def(
            nb::init<
                const std::vector<double> &,
                const std::vector<double> &,
                const std::vector<double> &,
                const std::vector<double> &,
                const std::vector<bool> &>(),
            nb::arg("a"),
            nb::arg("alpha"),
            nb::arg("d"),
            nb::arg("theta"),
            nb::arg("is_joint_revolute"),
            "Construct a DHKinematics object using separate parameter vectors\n\n"
            "Args:\n"
            "    a: Vector of link lengths\n"
            "    alpha: Vector of link twists\n"
            "    d: Vector of link offsets\n"
            "    theta: Vector of joint angles\n"
            "    is_joint_revolute: Vector indicating if each joint is revolute (True) or prismatic (False)")
        .def(
            nb::init<
                const Eigen::VectorXd &,
                const Eigen::VectorXd &,
                const Eigen::VectorXd &,
                const Eigen::VectorXd &,
                const std::vector<bool> &>(),
            nb::arg("a"),
            nb::arg("alpha"),
            nb::arg("d"),
            nb::arg("theta"),
            nb::arg("is_joint_revolute"),
            "Construct a DHKinematics object using Eigen vectors\n\n"
            "Args:\n"
            "    a: Eigen vector of link lengths\n"
            "    alpha: Eigen vector of link twists\n"
            "    d: Eigen vector of link offsets\n"
            "    theta: Eigen vector of joint angles\n"
            "    is_joint_revolute: Vector indicating if each joint is revolute (True) or prismatic (False)")
        .def(
            "get_a",
            &kinematics::DHKinematics::get_a,
            "Get the link length parameters\n\n"
            "Returns:\n"
            "    List of link lengths")
        .def(
            "get_alpha",
            &kinematics::DHKinematics::get_alpha,
            "Get the link twist parameters\n\n"
            "Returns:\n"
            "    List of link twists")
        .def(
            "get_d",
            &kinematics::DHKinematics::get_d,
            "Get the link offset parameters\n\n"
            "Returns:\n"
            "    List of link offsets")
        .def(
            "get_theta",
            &kinematics::DHKinematics::get_theta,
            "Get the joint angle parameters\n\n"
            "Returns:\n"
            "    List of joint angles");

    return m;
  }
}  // namespace sdu_controllers
