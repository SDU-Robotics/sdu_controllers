#include <yaml-cpp/yaml.h>

#include <iostream>
#include <sdu_controllers/models/parameter_robot_model.hpp>
#include <sdu_controllers/models/breeding_blanket_handling_robot_model.hpp>
#include <sdu_controllers/utils/utility.hpp>

using namespace Eigen;

namespace sdu_controllers::models
{
  BreedingBlanketHandlingRobotModel::BreedingBlanketHandlingRobotModel()
    : BreedingBlanketHandlingRobotModel(utils::project_path("config/models/breeding_blanket_handling_robot.yaml"))
  {
  }

  BreedingBlanketHandlingRobotModel::BreedingBlanketHandlingRobotModel(const std::string &yaml_filepath) : ParameterRobotModel(yaml_filepath)
  {
    // Initialize default mass, com and link inertia
    mass_default_ = this->get_m();
    com_default_ = this->get_CoM();
    link_inertia_default_ = this->get_link_inertia();
  }

  BreedingBlanketHandlingRobotModel::BreedingBlanketHandlingRobotModel(const RobotParameters &params) : ParameterRobotModel(params)
  {
    // Initialize default mass, com and link inertia
    mass_default_ = this->get_m();
    com_default_ = this->get_CoM();
    link_inertia_default_ = this->get_link_inertia();
  }

  void BreedingBlanketHandlingRobotModel::set_tcp_mass(double mass, Eigen::Vector3d& com, Eigen::Matrix3d inertia)
  {
    // self._m[5] = self._m_default[5] + mass
    // com_new = (numpy.asarray(self._com_default[5]) * self._m_default[5] + numpy.asarray(com) * mass)/self._m[5]
    // self._com[5] = com_new.tolist()
    // r = -numpy.asarray(self._com_default[5]) + com_newset_tcp_mass
    // offset_matrix_a = numpy.asarray([[r[1] ** 2 + r[2] ** 2, -r[0]*r[1],             -r[0]*r[2]],
    //                                  [-r[0]*r[1],             r[0] ** 2 + r[2] ** 2, -r[1]*r[2]],
    //                                  [-r[0]*r[2],            -r[1]*r[2],              r[0] ** 2 + r[1] ** 2]])
    // r = -numpy.asarray(com) + com_new
    // offset_matrix_b = numpy.asarray([[r[1] ** 2 + r[2] ** 2, -r[0]*r[1],             -r[0]*r[2]],
    //                                  [-r[0]*r[1],             r[0] ** 2 + r[2] ** 2, -r[1]*r[2]],
    //                                  [-r[0]*r[2],            -r[1]*r[2],              r[0] ** 2 + r[1] ** 2]])
    // new_inertia = numpy.asarray(self._inertia_default[5]) + self._m_default[5] * offset_matrix_a +\
    //               numpy.asarray(inertia) + mass * offset_matrix_b
    // self._inertia[5] = new_inertia.tolist()
    int N = get_dof();

    // Initialize new containers by copying defaults so we don't index uninitialized containers
    std::vector<double> new_mass = mass_default_;
    Eigen::Matrix<double, Eigen::Dynamic, 3> new_com = com_default_;
    std::vector<Eigen::Matrix3d> new_link_inertia = link_inertia_default_;

    // Ensure correct sizes (defensive)
    if ((int)new_mass.size() < N) new_mass.resize(N, 0.0);
    if (new_com.rows() < N) new_com.conservativeResize(N, 3);
    if ((int)new_link_inertia.size() < N) new_link_inertia.resize(N, Eigen::Matrix3d::Zero());

    // Update mass for the TCP (last link)
    new_mass[N - 1] = mass_default_[N - 1] + mass;

    // Compute new center of mass for the last link
    Eigen::Vector3d com_default_N = com_default_.row(N - 1).transpose();
    Eigen::Vector3d com_new = (com_default_N * mass_default_[N - 1] + com * mass) / new_mass[N - 1];
    new_com.row(N - 1) = com_new.transpose();

    // Compute offsets for inertia
    Eigen::Vector3d r = -com_default_N + com_new;

    Eigen::Matrix3d offset_matrix_a, offset_matrix_b;

    offset_matrix_a << pow(r(1), 2) + pow(r(2), 2), -r(0) * r(1), -r(0) * r(2),
        -r(0) * r(1), pow(r(0), 2) + pow(r(2), 2), -r(1) * r(2),
        -r(0) * r(2), -r(1) * r(2), pow(r(0), 2) + pow(r(1), 2);

    r = -com + com_new;
    offset_matrix_b << pow(r(1), 2) + pow(r(2), 2), -r(0) * r(1), -r(0) * r(2),
        -r(0) * r(1), pow(r(0), 2) + pow(r(2), 2), -r(1) * r(2),
        -r(0) * r(2), -r(1) * r(2), pow(r(0), 2) + pow(r(1), 2);

    Eigen::Matrix3d new_inertia =
        link_inertia_default_[N - 1] + mass_default_[N - 1] * offset_matrix_a + inertia + mass * offset_matrix_b;

    new_link_inertia[N - 1] = new_inertia;

    // Apply updated parameters
    this->set_mass(new_mass);
    this->set_com(new_com);
    this->set_link_inertia(new_link_inertia);
  }

}  // namespace sdu_controllers::models
