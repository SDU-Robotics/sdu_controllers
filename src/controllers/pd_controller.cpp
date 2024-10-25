#include <sdu_controllers/controllers/pd_controller.hpp>
#include <utility>

using namespace Eigen;

namespace sdu_controllers::controllers
{
  PDController::PDController(MatrixXd Kp, MatrixXd Kd, MatrixXd N) : Kp_(std::move(Kp)), Kd_(std::move(Kd)), N_(std::move(N))
  {
  }

  void
  PDController::step(const VectorXd &q_d, const VectorXd &dq_d, const VectorXd &u_ff, const VectorXd &q, const VectorXd &dq)
  {
    u_ = N_ * u_ff + Kp_ * (q_d - q) + Kd_ * (dq_d - dq);
  }

  void PDController::reset()
  {
    Kp_.setZero();
    Kd_.setZero();
    N_.setIdentity();
    u_.setZero();
  }

  VectorXd PDController::get_output()
  {
    return u_;
  }

}  // namespace sdu_controllers::controllers
