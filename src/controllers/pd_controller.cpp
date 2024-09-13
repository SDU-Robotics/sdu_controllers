#include <sdu_controllers/controllers/pd_controller.hpp>
#include <iostream>

using namespace Eigen;

namespace sdu_controllers::controllers
{
  PDController::PDController(const VectorXd &Kp, const VectorXd &Kd)
  {
    Kp_ = Kp;
    Kd_ = Kd;
  }

  PDController::~PDController() = default;

  void
  PDController::step(const VectorXd &q_d, const VectorXd &dq_d, const VectorXd &ddq_d, const VectorXd &q, const VectorXd &dq)
  {
    VectorXd position_error = q_d - q;
    VectorXd velocity_error = dq_d - dq;
    VectorXd Kp_e = Kp_.array() * position_error.array();
    VectorXd Kd_e = Kd_.array() * velocity_error.array();
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
