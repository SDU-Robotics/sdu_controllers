#include <sdu_controllers/controllers/operational_space_controller.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/math/math.hpp>
#include <utility>
#include <Eigen/src/QR/FullPivHouseholderQR.h>

using namespace Eigen;

namespace sdu_controllers::controllers
{
  OperationalSpaceController::OperationalSpaceController(
      MatrixXd Kp,
      MatrixXd Kd,
      std::shared_ptr<models::RobotModel> robot_model)
      : Kp_(std::move(Kp)),
        Kd_(std::move(Kd)),
        robot_model_(std::move(robot_model))
  {
    kappa_ = 0.0;
    Kd_null_ = 5.0;
  }

  void OperationalSpaceController::step(
      const VectorXd &x_d,
      const VectorXd &dx_d,
      const VectorXd &ddx_d,
      const VectorXd &q,
      const VectorXd &dq)
  {
    Matrix4d T = robot_model_->get_fk_solver().forward_kinematics(q);
    VectorXd pos = T.block<3, 1>(0,3);

    Matrix3d rot_mat = T.topLeftCorner(3, 3);

    VectorXd rot_zyz = rot_mat.eulerAngles(2, 1, 2); // ZYZ representation

    VectorXd x_e(pos.size() + rot_zyz.size());
    x_e << pos, rot_zyz;

    VectorXd x_tilde = x_d - x_e;

    MatrixXd J_A = math::jacobian_analytical(q, robot_model_);

    MatrixXd Jdot_A = math::jacobian_dot_analytical(q, dq, robot_model_);

    VectorXd dx_e = J_A * dq;

    VectorXd dx_tilde = dx_d - dx_e;

    // Eq. (8.114) from page 348, Robotics: Modelling, Planning and Control:
    Eigen::MatrixXd kappaI;
    kappaI.setIdentity(6, 6);
    kappaI *= kappa_*kappa_; // todo: should be user configurable

    // Explicitly calculate the DLS pseudo-inverse, see eq. (3.59)
    Eigen::MatrixXd J_pinv = J_A.transpose() * 
        (J_A * J_A.transpose() + kappaI).fullPivHouseholderQr().solve(Eigen::MatrixXd::Identity(6, 6));

    // Calculate the primary task command
    Eigen::VectorXd task_cmd = ddx_d + Kd_ * dx_tilde + Kp_ * x_tilde - Jdot_A * dq;

    // Project primary task into joint space
    Eigen::VectorXd y_primary = J_pinv * task_cmd;

    // Calculate the null-space projection matrix: N = I - J_pinv * J_A
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd N = I - J_pinv * J_A;

    // Secondary task (Joint Damping to stop drift)
    Eigen::VectorXd secondary_cmd = -Kd_null_ * dq; 

    // Combine primary and secondary tasks (see eq. 3.54)
    y_ = y_primary + N * secondary_cmd;
  }

  void OperationalSpaceController::reset()
  {
    Kp_.setZero();
    Kd_.setZero();
    y_.setZero();
    kappa_ = 0.0;
    Kd_null_ = 5.0;
  }

  VectorXd OperationalSpaceController::get_output()
  {
    return y_;
  }

  void OperationalSpaceController::set_kappa(double kappa)
  {
    kappa_ = kappa;
  }

  void OperationalSpaceController::set_Kd_null(double Kd_null)
  {
    Kd_null_ = Kd_null;
  }

}  // namespace sdu_controllers::controllers
