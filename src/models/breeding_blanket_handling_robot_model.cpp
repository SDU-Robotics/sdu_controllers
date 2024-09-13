#include <iostream>
#include <sdu_controllers/models/breeding_blanket_handling_robot_model.hpp>

using namespace Eigen;

namespace sdu_controllers::models
{
  BreedingBlanketHandlingRobotModel::BreedingBlanketHandlingRobotModel() : RobotModel()
  {
  }

  BreedingBlanketHandlingRobotModel::~BreedingBlanketHandlingRobotModel() = default;

  MatrixXd BreedingBlanketHandlingRobotModel::get_inertia_matrix(const VectorXd& q)
  {
    MatrixXd I = MatrixXd::Identity(ROBOT_DOF, ROBOT_DOF);
    return I;
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_coriolis(const VectorXd& q, const VectorXd& qd)
  {
    MatrixXd I = MatrixXd::Identity(ROBOT_DOF, ROBOT_DOF);
    return I;
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_gravity(const VectorXd& q)
  {
    VectorXd v = VectorXd::Zero(ROBOT_DOF);
    return v;
  }

  uint16_t BreedingBlanketHandlingRobotModel::get_dof() const
  {
    return dof_;
  }


}  // namespace sdu_controllers::math
