#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include <sdu_controllers/controllers/admittance_controller_position.hpp>
#include <sdu_controllers/controllers/pid_controller.hpp>
#include <sdu_controllers/math/forward_dynamics.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/math/rnea.hpp>
#include <sdu_controllers/models/breeding_blanket_handling_robot_model.hpp>
#include <sdu_controllers/models/robot_model.hpp>
#include <sdu_controllers/models/ur_robot.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>
#include <sdu_controllers/sdu_controllers.hpp>

namespace nb = nanobind;

namespace sdu_controllers
{
  NB_MODULE(_sdu_controllers, m)
  {
    m.doc() = "Python Bindings for sdu_controllers";
    m.def("add_one", &add_one, "Increments an integer value");

    // enums
    nb::enum_<URRobot::RobotType>(m, "RobotType")
        .value("UR3e", URRobot::RobotType::UR3e)
        .value("UR5e", URRobot::RobotType::UR5e)
        .export_values();

    // models
    nb::class_<models::RobotModel>(m, "RobotModel");

    nb::class_<models::URRobotModel, models::RobotModel>(m, "URRobotModel")
        .def(nb::init<>())
        .def("get_inertia_matrix", &models::URRobotModel::get_inertia_matrix)
        .def("get_coriolis", &models::URRobotModel::get_coriolis)
        .def("get_gravity", &models::URRobotModel::get_gravity)
        .def("get_dof", &models::URRobotModel::get_dof)
        .def("get_joint_pos_bounds", &models::URRobotModel::get_joint_pos_bounds)
        .def("get_joint_vel_bounds", &models::URRobotModel::get_joint_vel_bounds)
        .def("get_joint_acc_bounds", &models::URRobotModel::get_joint_acc_bounds)
        .def("get_joint_torque_bounds", &models::URRobotModel::get_joint_torque_bounds);

    nb::class_<models::BreedingBlanketHandlingRobotModel, models::RobotModel>(m, "BreedingBlanketHandlingRobotModel")
        .def(nb::init<>())
        .def("get_inertia_matrix", &models::BreedingBlanketHandlingRobotModel::get_inertia_matrix)
        .def("get_coriolis", &models::BreedingBlanketHandlingRobotModel::get_coriolis)
        .def("get_gravity", &models::BreedingBlanketHandlingRobotModel::get_gravity)
        .def("get_dof", &models::BreedingBlanketHandlingRobotModel::get_dof);

    // controllers
    nb::class_<controllers::PIDController>(m, "PIDController")
        .def(
            nb::init<const Eigen::MatrixXd &, const Eigen::MatrixXd &, 
                         const Eigen::MatrixXd &, const Eigen::MatrixXd &,
                         double>(),
            nb::arg("Kp"),
            nb::arg("Ki"),
            nb::arg("Kd"),
            nb::arg("N"),
            nb::arg("dt"))
        .def("step", &controllers::PIDController::step)
        .def("get_output", &controllers::PIDController::get_output)
        .def("reset", &controllers::PIDController::reset);

    nb::class_<controllers::AdmittanceControllerPosition>(m, "AdmittanceControllerPosition")
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

    // math
    nb::class_<math::InverseDynamicsJointSpace>(m, "InverseDynamicsJointSpace")
        .def(nb::init<std::shared_ptr<models::RobotModel>>())
        .def("inverse_dynamics", &math::InverseDynamicsJointSpace::inverse_dynamics);

    nb::class_<math::ForwardDynamics>(m, "ForwardDynamics")
        .def(nb::init<std::shared_ptr<models::RobotModel>>())
        .def("forward_dynamics", &math::ForwardDynamics::forward_dynamics);

    // Recursive Newton-Euler Algorithm
    nb::module_ eigen = nb::module_::import_("numpy");
    nb::class_<sdu_controllers::math::RecursiveNewtonEuler>(m, "RecursiveNewtonEuler")
        .def(
            nb::init<std::shared_ptr<sdu_controllers::models::RobotModel>>(),
            "Initialize the RecursiveNewtonEuler algorithm with a robot model",
            nb::arg("robot_model"))
        .def(
            "inverse_dynamics",
            &sdu_controllers::math::RecursiveNewtonEuler::inverse_dynamics,
            "Compute inverse dynamics: τ = H(q)q̈ + C(q,q̇)q̇ + g(q)",
            nb::arg("q"),
            nb::arg("dq"),
            nb::arg("ddq"),
            nb::arg("he"))
        .def(
            "forward_dynamics",
            &sdu_controllers::math::RecursiveNewtonEuler::forward_dynamics,
            "Compute forward dynamics: q̈ = H(q)⁻¹(τ - C(q,q̇)q̇ - g(q))",
            nb::arg("q"),
            nb::arg("dq"),
            nb::arg("tau"),
            nb::arg("he"))
        .def(
            "inertia",
            &sdu_controllers::math::RecursiveNewtonEuler::inertia,
            "Compute the joint-space inertia matrix H(q)",
            nb::arg("q"))
        .def(
            "velocity_product",
            &sdu_controllers::math::RecursiveNewtonEuler::velocityProduct,
            "Compute the velocity product term C(q,q̇)q̇ from the manipulator equation",
            nb::arg("q"),
            nb::arg("dq"))
        .def(
            "gravity",
            &sdu_controllers::math::RecursiveNewtonEuler::gravity,
            "Compute the gravity forces g(q)",
            nb::arg("q"))
        .def(
            "set_z0",
            &sdu_controllers::math::RecursiveNewtonEuler::set_z0,
            "Set the z-axis of the base frame",
            nb::arg("z0"));
  }

}  // namespace sdu_controllers
