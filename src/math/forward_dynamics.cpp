#include <iostream>
#include <sdu_controllers/math/forward_dynamics.hpp>
#include <utility>

using namespace Eigen;

namespace sdu_controllers::math
{
  ForwardDynamics::ForwardDynamics(std::shared_ptr<models::RobotModel> robot_model) : robot_model_(std::move(robot_model)),
        dt_(1.0/10.0)
  {
    dq_e_.resize(6);
    q_e_.resize(6);
    dq_e_.setZero();
    q_e_.setZero();
  }

  VectorXd ForwardDynamics::forward_dynamics(const VectorXd& q, const VectorXd& dq, const VectorXd& tau, const VectorXd& q_d)
  {
    const std::shared_ptr<models::RobotModel> robot_model = get_robot_model();
    MatrixXd B = robot_model->get_inertia_matrix(q);
    MatrixXd C = robot_model->get_coriolis(q, dq);
    VectorXd tau_g = robot_model->get_gravity(q);

    // Eq. (7.115), from page 293, Robotics: Modelling, Planning and Control.
    VectorXd tau_mark = C * dq; // + tau_g
    VectorXd ddq_e = B.inverse() * (tau - tau_mark);

    std::cout << "ddq_e:" << ddq_e << std::endl;

    // integrate to get velocity error
    dq_e_ += ddq_e * dt_;

    // integrate to get position error
    q_e_ += dq_e_ * dt_;

    // add desired joing position q_d
    VectorXd q_out = q_d - q_e_;
    return q_out;
  }

}  // namespace sdu_controllers::math
