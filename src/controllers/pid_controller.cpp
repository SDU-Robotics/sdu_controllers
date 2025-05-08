#include <sdu_controllers/controllers/pid_controller.hpp>
#include <utility>
#include <iostream>

using namespace Eigen;

namespace sdu_controllers::controllers
{
  PIDController::PIDController(MatrixXd Kp, MatrixXd Ki, MatrixXd Kd, MatrixXd N, double dt) 
    : Kp_(std::move(Kp)), Ki_(std::move(Ki)), Kd_(std::move(Kd)), N_(std::move(N)), dt(std::move(dt))
  {
    integral_term.resize(Kp_.cols());
    integral_term.setZero();
  }

  void
  PIDController::step(const VectorXd &q_d, const VectorXd &dq_d, const VectorXd &u_ff, 
                      const VectorXd &q, const VectorXd &dq)
  {
    integral_term += dt * (q_d - q);

    u_ = N_ * u_ff + Kp_ * (q_d - q) + Ki_ * integral_term + Kd_ * (dq_d - dq);
  }

  void PIDController::reset()
  {
    Kp_.setZero();
    Ki_.setZero();
    Kd_.setZero();
    N_.setIdentity();
    u_.setZero();
    integral_term.setZero();
  }

  VectorXd PIDController::get_output()
  {
    return u_;
  }

}  // namespace sdu_controllers::controllers
