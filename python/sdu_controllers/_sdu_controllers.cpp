#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

// controllers
#include <sdu_controllers/controllers/admittance_controller_position.hpp>
#include <sdu_controllers/controllers/force_control_inner_velocity_loop.hpp>
#include <sdu_controllers/controllers/operational_space_controller.hpp>
#include <sdu_controllers/controllers/pid_controller.hpp>

// kinematics
#include <sdu_controllers/kinematics/dh_kinematics.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/sdu_controllers.hpp>

namespace nb = nanobind;

namespace sdu_controllers
{
  nb::module_ create_robot_models_module(nb::module_ &main_module);
  nb::module_ create_math_module(nb::module_ &main_module);

  NB_MODULE(_sdu_controllers, m)
  {
    m.doc() = "Python Bindings for sdu_controllers";

    nb::module_ m_models = create_robot_models_module(m);
    nb::module_ m_controllers = m.def_submodule("controllers", "Submodule containing robot control methods.");
    nb::module_ m_math = create_math_module(m);
    nb::module_ m_kinematics = m.def_submodule("kinematics", "Submodule containing functions for calculating kinematics.");

    // controllers
    nb::class_<controllers::PIDController>(m_controllers, "PIDController")
        .def(
            nb::init<
                const Eigen::MatrixXd &,
                const Eigen::MatrixXd &,
                const Eigen::MatrixXd &,
                const Eigen::MatrixXd &,
                double,
                const Eigen::VectorXd &,
                const Eigen::VectorXd &>(),
            nb::arg("Kp"),
            nb::arg("Ki"),
            nb::arg("Kd"),
            nb::arg("N"),
            nb::arg("dt"),
            nb::arg("u_min"),
            nb::arg("u_max"))
        .def("step", &controllers::PIDController::step)
        .def("get_output", &controllers::PIDController::get_output)
        .def("reset", &controllers::PIDController::reset);

    nb::class_<controllers::AdmittanceControllerPosition>(m_controllers, "AdmittanceControllerPosition")
        .def(nb::init<const double>(), nb::arg("frequency"))
        .def("step", &controllers::AdmittanceControllerPosition::step)
        .def("get_output", &controllers::AdmittanceControllerPosition::get_output)
        .def("reset", &controllers::AdmittanceControllerPosition::reset)
        .def("set_mass_matrix_position", &controllers::AdmittanceControllerPosition::set_mass_matrix_position)
        .def("set_mass_matrix_orientation", &controllers::AdmittanceControllerPosition::set_mass_matrix_orientation)
        .def("set_stiffness_matrix_position", &controllers::AdmittanceControllerPosition::set_stiffness_matrix_position)
        .def(
            "set_stiffness_matrix_orientation", &controllers::AdmittanceControllerPosition::set_stiffness_matrix_orientation)
        .def("set_damping_matrix_position", &controllers::AdmittanceControllerPosition::set_damping_matrix_position)
        .def("set_damping_matrix_orientation", &controllers::AdmittanceControllerPosition::set_damping_matrix_orientation);

    // Cartesian motion controller
    nb::class_<controllers::OperationalSpaceController>(m_controllers, "OperationalSpaceController")
        .def(
            nb::init<const Eigen::MatrixXd &, const Eigen::MatrixXd &, std::shared_ptr<models::RobotModel>>(),
            nb::arg("Kp"),
            nb::arg("Kd"),
            nb::arg("robot_model"))
        .def("step", &controllers::OperationalSpaceController::step)
        .def("get_output", &controllers::OperationalSpaceController::get_output)
        .def("reset", &controllers::OperationalSpaceController::reset);

    // Force controller, velocity based
    nb::class_<controllers::ForceControlInnerVelocityLoop>(m_controllers, "ForceControlInnerVelocityLoop")
        .def(
            nb::init<
                const Eigen::MatrixXd &,
                const Eigen::MatrixXd &,
                const Eigen::MatrixXd &,
                const Eigen::MatrixXd &,
                std::shared_ptr<models::RobotModel>>(),
            nb::arg("Kp"),
            nb::arg("Kd"),
            nb::arg("Md"),
            nb::arg("Kf"),
            nb::arg("robot_model"))
        .def("step", &controllers::ForceControlInnerVelocityLoop::step)
        .def("get_output", &controllers::ForceControlInnerVelocityLoop::get_output)
        .def("reset", &controllers::ForceControlInnerVelocityLoop::reset);

    nb::enum_<kinematics::ForwardKinematics::JointType>(m_kinematics, "JointType")
        .value("REVOLUTE", kinematics::ForwardKinematics::JointType::REVOLUTE)
        .value("PRISMATIC", kinematics::ForwardKinematics::JointType::PRISMATIC)
        .export_values();

    nb::class_<sdu_controllers::kinematics::ForwardKinematics>(m_kinematics, "ForwardKinematics")
        .def(
            "forward_kinematics",
            &sdu_controllers::kinematics::ForwardKinematics::forward_kinematics,
            "Get the transformation matrix from base to end-effector",
            nb::arg("q"))
        .def(
            "forward_kinematics_all",
            &sdu_controllers::kinematics::ForwardKinematics::forward_kinematics_all,
            "Get the transformation matrices from base to each joint frame",
            nb::arg("q"))
        .def(
            "get_joint_types",
            &sdu_controllers::kinematics::ForwardKinematics::get_joint_types,
            "Get the type of each joint in the kinematic chain")
        .def(
            "geometric_jacobian",
            nb::overload_cast<const Eigen::VectorXd &>(
                &sdu_controllers::kinematics::ForwardKinematics::geometric_jacobian, nb::const_),
            "Compute the geometric Jacobian at the given joint configuration",
            nb::arg("q"))
        .def(
            "geometric_jacobian",
            nb::overload_cast<const Eigen::VectorXd &, const std::vector<Eigen::Matrix4d> &>(
                &sdu_controllers::kinematics::ForwardKinematics::geometric_jacobian, nb::const_),
            "Compute the geometric Jacobian at the given joint configuration using precomputed forward kinematics matrices",
            nb::arg("q"),
            nb::arg("fk_matrices"))
        .def(
            "get_dof",
            &sdu_controllers::kinematics::ForwardKinematics::get_dof,
            "Get the degrees of freedom of the kinematic chain");

    nb::class_<sdu_controllers::kinematics::DHKinematics, sdu_controllers::kinematics::ForwardKinematics>(
        m_kinematics, "DHKinematics")
        .def(nb::init<>())
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
            nb::arg("is_joint_revolute"))
        .def("get_a", &sdu_controllers::kinematics::DHKinematics::get_a)
        .def("get_alpha", &sdu_controllers::kinematics::DHKinematics::get_alpha)
        .def("get_d", &sdu_controllers::kinematics::DHKinematics::get_d)
        .def("get_theta", &sdu_controllers::kinematics::DHKinematics::get_theta);
  }

}  // namespace sdu_controllers
