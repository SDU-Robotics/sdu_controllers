#include <iostream>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <utility>

using namespace Eigen;

namespace sdu_controllers::math
{
  InverseDynamicsJointSpace::InverseDynamicsJointSpace(std::shared_ptr<models::RobotModel> robot_model)
      : InverseDynamics(std::move(robot_model))
  {
  }

  VectorXd InverseDynamicsJointSpace::inverse_dynamics(const VectorXd& y, const VectorXd& q, const VectorXd& dq)
  {
    const std::shared_ptr<models::RobotModel> robot_model = get_robot_model();
    MatrixXd B = robot_model->get_inertia_matrix(q);
    MatrixXd C = robot_model->get_coriolis(q, dq);
    VectorXd tau_g = robot_model->get_gravity(q);

    // Eq. (6.27) from page 141, Springer Handbook of Robotics 2008.
    VectorXd tau = B * y + C * dq + tau_g;

    return tau;
  }

}  // namespace sdu_controllers::math