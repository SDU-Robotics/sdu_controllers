#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include <sdu_controllers/kinematics/forward_kinematics.hpp>

// models
#include <sdu_controllers/models/breeding_blanket_handling_robot_model.hpp>
#include <sdu_controllers/models/parameter_robot_model.hpp>
#include <sdu_controllers/models/regressor_robot_model.hpp>
#include <sdu_controllers/models/robot_model.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>

namespace nb = nanobind;

namespace sdu_controllers
{

  nb::module_ create_robot_models_module(nb::module_ &main_module)
  {
    nb::module_ m_models = main_module.def_submodule("models", "Submodule containing robot model definitions.");
    // enums
    nb::enum_<models::URRobotModel::RobotType>(m_models, "RobotType")
        .value("ur3e", models::URRobotModel::RobotType::ur3e)
        .value("ur5e", models::URRobotModel::RobotType::ur5e)
        .value("ur10e", models::URRobotModel::RobotType::ur10e)
        .export_values();

    // models

    nb::class_<models::RobotModel>(m_models, "RobotModel")
        .def(
            "inverse_dynamics",
            &models::RobotModel::inverse_dynamics,
            nb::arg("q"),
            nb::arg("dq"),
            nb::arg("ddq"),
            nb::arg("he"))
        .def("forward_dynamics", &models::RobotModel::forward_dynamics, nb::arg("q"), nb::arg("dq"), nb::arg("tau"))
        .def("get_inertia_matrix", &models::RobotModel::get_inertia_matrix)
        .def("get_coriolis", &models::RobotModel::get_coriolis)
        .def("get_gravity", &models::RobotModel::get_gravity)
        .def("get_jacobian", &models::RobotModel::get_jacobian, nb::arg("q"))
        .def("get_jacobian_dot", &models::RobotModel::get_jacobian_dot, nb::arg("q"), nb::arg("dq"))
        .def("get_joint_pos_bounds", &models::RobotModel::get_joint_pos_bounds)
        .def("get_joint_max_vel", &models::RobotModel::get_joint_max_vel)
        .def("get_joint_max_acc", &models::RobotModel::get_joint_max_acc)
        .def("get_joint_max_torque", &models::RobotModel::get_joint_max_torque)
        .def("get_dof", &models::RobotModel::get_dof)
        .def("get_m", &models::RobotModel::get_m)
        .def("get_g0", &models::RobotModel::get_g0)
        .def("get_link_inertia", &models::RobotModel::get_link_inertia)
        .def("get_fk_solver", &models::RobotModel::get_fk_solver);

    nb::class_<models::RegressorRobotModel, models::RobotModel>(m_models, "RegressorRobotModel")
        .def("get_regressor", &models::RegressorRobotModel::get_regressor, nb::arg("q"), nb::arg("qd"), nb::arg("qdd"))
        .def(
            "get_friction_regressor",
            nb::overload_cast<const Eigen::VectorXd &>(&models::RegressorRobotModel::get_friction_regressor, nb::const_),
            nb::arg("qd"))
        .def(
            "get_friction_regressor",
            nb::overload_cast<const std::vector<double> &>(&models::RegressorRobotModel::get_friction_regressor, nb::const_),
            nb::arg("qd"))
        .def("get_parameters", &models::RegressorRobotModel::get_parameters)
        .def("get_friction_parameters", &models::RegressorRobotModel::get_friction_parameters);

    nb::class_<models::ParameterRobotModel>(m_models, "ParameterRobotModel");

    nb::class_<models::URRobotModel, models::ParameterRobotModel>(m_models, "URRobotModel")
        .def(nb::init<models::URRobotModel::RobotType>())
        .def(nb::init<const std::string &>())
        .def(nb::init<const models::RobotParameters &>());

    nb::class_<models::BreedingBlanketHandlingRobotModel, models::ParameterRobotModel>(
        m_models, "BreedingBlanketHandlingRobotModel")
        .def(nb::init<>())
        .def(nb::init<const std::string &>())
        .def(nb::init<const models::RobotParameters &>());
    return m_models;
  }
}  // namespace sdu_controllers