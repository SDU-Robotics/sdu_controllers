#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>

using namespace Eigen;

namespace sdu_controllers::math
{
  InverseDynamicsJointSpace::InverseDynamicsJointSpace(const models::RobotModel& robot_model) : InverseDynamics(robot_model)
  {
  }

  InverseDynamicsJointSpace::~InverseDynamicsJointSpace() = default;

  VectorXd InverseDynamicsJointSpace::inverse_dynamics(
      const VectorXd& y,
      const VectorXd& q,
      const VectorXd& qd)
  {
    MatrixXd B = robot_model_.get_inertia_matrix(q);
    MatrixXd C = robot_model_.get_coriolis(q, qd);
    VectorXd tau_g = robot_model_.get_gravity(q);

    // Eq. (6.27) from page 141, Springer Handbook of Robotics 2008.
    VectorXd tau = B * y + C * qd + tau_g;

    return tau;
  }

}  // namespace sdu_controllers::math