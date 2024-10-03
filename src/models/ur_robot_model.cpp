#include <sdu_controllers/models/ur_robot_model.hpp>
#include <sdu_controllers/models/ur_robot.hpp>

using namespace Eigen;

namespace sdu_controllers::models
{
  URRobotModel::URRobotModel(RobotType robot_type) : RobotModel(), ur_robot_(robot_type)
  {
  }

  MatrixXd URRobotModel::get_inertia_matrix(const VectorXd& q)
  {
    return ur_robot_.inertia(q);
  }

  MatrixXd URRobotModel::get_coriolis(const VectorXd& q, const VectorXd& qd)
  {
    return ur_robot_.coriolis(q, qd);
  }

  MatrixXd URRobotModel::get_gravity(const VectorXd& q)
  {
    return ur_robot_.gravity(q);
  }

  //std::pair<VectorXd, VectorXd> get_bounds()
  //{
  //}

  uint16_t URRobotModel::get_dof() const
  {
    return dof_;
  }

}  // namespace sdu_controllers::models
