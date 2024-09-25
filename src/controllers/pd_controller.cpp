#include <sdu_controllers/controllers/pd_controller.hpp>
#include <utility>

using namespace Eigen;

namespace sdu_controllers::controllers
{
  PDController::PDController(MatrixXd Kp, MatrixXd Kd) : Kp_(std::move(Kp)), Kd_(std::move(Kd))
  {
  }

  void
  PDController::step(const VectorXd &q_d, const VectorXd &dq_d, const VectorXd &ddq_d, const VectorXd &q, const VectorXd &dq)
  {
    const VectorXd Kp_e = Kp_ * (q_d - q);
    const VectorXd Kd_e = Kd_ * (dq_d - dq);
    u_ = ddq_d + Kp_e + Kd_e;
  }

  void PDController::reset()
  {
    Kp_.setZero();
    Kd_.setZero();
    u_.setZero();
  }

  VectorXd PDController::get_output()
  {
    return u_;
  }

}  // namespace sdu_controllers::controllers
