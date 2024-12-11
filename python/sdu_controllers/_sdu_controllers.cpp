#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <sdu_controllers/controllers/pd_controller.hpp>
#include <sdu_controllers/controllers/admittance_controller_position.hpp>
#include <sdu_controllers/math/forward_dynamics.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
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

    nb::class_<models::BreedingBlanketHandlingRobotModel, models::RobotModel>(
        m, "BreedingBlanketHandlingRobotModel")
        .def(nb::init<>())
        .def("get_inertia_matrix", &models::BreedingBlanketHandlingRobotModel::get_inertia_matrix)
        .def("get_coriolis", &models::BreedingBlanketHandlingRobotModel::get_coriolis)
        .def("get_gravity", &models::BreedingBlanketHandlingRobotModel::get_gravity)
        .def("get_dof", &models::BreedingBlanketHandlingRobotModel::get_dof);

    // controllers
    nb::class_<controllers::PDController>(m, "PDController")
        .def(
            nb::init<const Eigen::MatrixXd &, const Eigen::MatrixXd &, const Eigen::MatrixXd &>(),
            nb::arg("Kp"),
            nb::arg("Kd"),
            nb::arg("N"))
        .def("step", &controllers::PDController::step)
        .def("get_output", &controllers::PDController::get_output)
        .def("reset", &controllers::PDController::reset);

    nb::class_<controllers::AdmittanceControllerPosition>(m, "AdmittanceControllerPosition")
        .def(nb::init<const double>(), nb::arg("frequency"))
        .def("step", &controllers::AdmittanceControllerPosition::step)
        .def("get_output", &controllers::AdmittanceControllerPosition::get_output)
        .def("reset", &controllers::AdmittanceControllerPosition::reset)
        .def("set_mass_matrix_position", &controllers::AdmittanceControllerPosition::set_mass_matrix_position)
        .def("set_mass_matrix_orientation", &controllers::AdmittanceControllerPosition::set_mass_matrix_orientation)
        .def("set_stiffness_matrix_position", &controllers::AdmittanceControllerPosition::set_stiffness_matrix_position)
        .def("set_stiffness_matrix_orientation", &controllers::AdmittanceControllerPosition::set_stiffness_matrix_orientation)
        .def("set_damping_matrix_position", &controllers::AdmittanceControllerPosition::set_damping_matrix_position)
        .def("set_damping_matrix_orientation", &controllers::AdmittanceControllerPosition::set_damping_matrix_orientation)
    ;

    // math
    nb::class_<math::InverseDynamicsJointSpace>(m, "InverseDynamicsJointSpace")
        .def(nb::init<std::shared_ptr<models::RobotModel>>())
        .def("inverse_dynamics", &math::InverseDynamicsJointSpace::inverse_dynamics);

    nb::class_<math::ForwardDynamics>(m, "ForwardDynamics")
        .def(nb::init<std::shared_ptr<models::RobotModel>>())
        .def("forward_dynamics", &math::ForwardDynamics::forward_dynamics);
  }

}  // namespace sdu_controllers
