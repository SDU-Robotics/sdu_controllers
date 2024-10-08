#include <iostream>
#include <sdu_controllers/math/forward_dynamics.hpp>
#include <utility>

using namespace Eigen;

namespace sdu_controllers::math
{
  ForwardDynamics::ForwardDynamics(std::shared_ptr<models::RobotModel> robot_model) :
        robot_model_(std::move(robot_model))
  {
  }

  VectorXd ForwardDynamics::forward_dynamics(const VectorXd& q, const VectorXd& dq, const VectorXd& tau)
  {
    const std::shared_ptr<models::RobotModel> robot_model = get_robot_model();
    MatrixXd B = robot_model->get_inertia_matrix(q);
    MatrixXd C = robot_model->get_coriolis(q, dq);
    VectorXd tau_g = robot_model->get_gravity(q);

    // Eq. (7.115), from page 293, Robotics: Modelling, Planning and Control.
    VectorXd tau_mark = C * dq + tau_g;
    VectorXd ddq = B.inverse() * (tau - tau_mark);
    std::cout << "ddq:" << ddq << std::endl;
    return ddq;
  }

}  // namespace sdu_controllers::math
