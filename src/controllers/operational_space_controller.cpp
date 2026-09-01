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
      std::shared_ptr<models::RobotModel> robot_model,
      OrientationRepresentation orientation_rep)
      : Kp_(std::move(Kp)),
        Kd_(std::move(Kd)),
        robot_model_(std::move(robot_model)),
        orientation_rep_(orientation_rep)
  {
    kappa_ = 0;
  }

  void OperationalSpaceController::step(
      const VectorXd &x_d,
      const VectorXd &dx_d,
      const VectorXd &ddx_d,
      const VectorXd &q,
      const VectorXd &dq,
      const Vector4d &quat_d)
  {
    // Eq. (8.114) from page 348, Robotics: Modelling, Planning and Control:
    Eigen::MatrixXd kappaI;
    kappaI.setIdentity(6, 6);
    kappaI *= kappa_*kappa_; // todo: should be user configurable

    if (orientation_rep_ == OrientationRepresentation::ZYZ)
    {
      // ---------------------------------------------------------------
      // ZYZ path - analytical Jacobian, Euler-angle orientation error
      // ---------------------------------------------------------------
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

      // The following implements damped least-squares, see eq. (3.59)
      y_ = J_A.transpose() * (J_A * J_A.transpose() + kappaI).fullPivHouseholderQr().solve(
        ddx_d + Kd_ * dx_tilde + Kp_ * x_tilde - Jdot_A * dq
      );
    }
    else
    {
      // ---------------------------------------------------------------
      // QUATERNION path - geometric Jacobian, quaternion orientation error
      // ---------------------------------------------------------------
      MatrixXd J    = robot_model_->get_jacobian(q);
      MatrixXd Jdot = robot_model_->get_jacobian_dot(q, dq);

      Matrix4d T   = robot_model_->get_fk_solver().forward_kinematics(q);
      Vector3d pos = T.block<3, 1>(0, 3);
      Matrix3d R   = T.topLeftCorner(3, 3);

      // Current orientation as unit quaternion
      Quaterniond q_e(R);
      q_e.normalize();
      double   eta_e = q_e.w();
      Vector3d eps_e = q_e.vec();

      // Desired orientation quaternion [w, x, y, z]
      Quaterniond q_d_quat(quat_d[0], quat_d[1], quat_d[2], quat_d[3]);
      q_d_quat.normalize();
      double   eta_d = q_d_quat.w();
      Vector3d eps_d = q_d_quat.vec();

      // Orientation error
      double eta_tilde = eta_d * eta_e + eps_d.dot(eps_e);
      Vector3d eps_tilde = eta_e * eps_d - eta_d * eps_e
                           - math::skew(eps_d) * eps_e;

      // Ensure shortest-arc
      if (eta_tilde < 0.0)
      {
        eps_tilde = -eps_tilde;
        eta_tilde = -eta_tilde;
      }

      // Position error
      Vector3d p_tilde = x_d.head<3>() - pos;

      VectorXd x_tilde(6);
      x_tilde << p_tilde, eps_tilde;

      // Velocity error
      VectorXd v_e = J * dq;
      VectorXd dx_tilde(6);
      dx_tilde << (dx_d.head<3>() - v_e.head<3>()),
                  (dx_d.tail<3>() - v_e.tail<3>());

      // The following implements damped least-squares, see eq. (3.59)
      y_ = J.transpose() * (J * J.transpose() + kappaI).fullPivHouseholderQr().solve(
        ddx_d + Kd_ * dx_tilde + Kp_ * x_tilde - Jdot * dq
      );
    }
  }

  void OperationalSpaceController::reset()
  {
    Kp_.setZero();
    Kd_.setZero();
    y_.setZero();
    kappa_ = 0;
  }

  VectorXd OperationalSpaceController::get_output()
  {
    return y_;
  }

  void OperationalSpaceController::set_kappa(double kappa)
  {
    kappa_ = kappa;
  }

}  // namespace sdu_controllers::controllers
