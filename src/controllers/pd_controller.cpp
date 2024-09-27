#include <sdu_controllers/controllers/pd_controller.hpp>
#include <utility>

using namespace Eigen;

namespace sdu_controllers::controllers
{
  PDController::PDController(MatrixXd Kp, MatrixXd Kd, MatrixXd N) : Kp_(std::move(Kp)), Kd_(std::move(Kd)), N_(std::move(N))
  {
  }

  void
  PDController::step(const VectorXd &q_d, const VectorXd &dq_d, const VectorXd &ddq_d, const VectorXd &q, const VectorXd &dq)
  {
    const VectorXd Kp_e = Kp_ * (q_d - q);
    const VectorXd Kd_e = Kd_ * (dq_d - dq);
    const VectorXd u_ff = N_ * ddq_d;
    u_ = u_ff + Kp_e + Kd_e;
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
