#include <sdu_controllers/models/breeding_blanket_handling_robot_model.hpp>

using namespace Eigen;

namespace sdu_controllers::models
{
  BreedingBlanketHandlingRobotModel::BreedingBlanketHandlingRobotModel() = default;

  BreedingBlanketHandlingRobotModel::~BreedingBlanketHandlingRobotModel() = default;

  MatrixXd BreedingBlanketHandlingRobotModel::get_inertia_matrix(const VectorXd& q)
  {
    MatrixXd I = MatrixXd::Identity(3, 3);
    return I;
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_coriolis(const VectorXd& q, const VectorXd& qd)
  {
    MatrixXd I = MatrixXd::Identity(3, 3);
    return I;
  }

  MatrixXd BreedingBlanketHandlingRobotModel::get_gravity(const VectorXd& q)
  {
    MatrixXd I = MatrixXd::Identity(3, 3);
    return I;
  }


}  // namespace sdu_controllers::math
