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
    kappa_ = 0.0;
    Kd_null_ = 5.0;
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
    kappaI *= kappa_ * kappa_; // todo: should be user configurable

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

      // Explicitly calculate the DLS pseudo-inverse, see eq. (3.59)
      Eigen::MatrixXd J_pinv = J_A.transpose() * 
          (J_A * J_A.transpose() + kappaI).fullPivHouseholderQr().solve(Eigen::MatrixXd::Identity(6, 6));

      // Calculate the primary task command
      Eigen::VectorXd task_cmd = ddx_d + Kd_ * dx_tilde + Kp_ * x_tilde - Jdot_A * dq;

      // Project primary task into joint space
      Eigen::VectorXd y_primary = J_pinv * task_cmd;

      // Calculate the null-space projection matrix: N = I - J_pinv * J_A

      const Eigen::Index n = robot_model_->get_dof();
      Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
      Eigen::MatrixXd N = I - J_pinv * J_A;

      // Secondary task (Joint Damping to stop drift)
      Eigen::VectorXd secondary_cmd = -Kd_null_ * dq; 

      // Combine primary and secondary tasks (see eq. 3.54)
      y_ = y_primary + N * secondary_cmd;
    }
    else
    {
      // ---------------------------------------------------------------
      // QUATERNION path - geometric Jacobian, quaternion orientation error
      // ---------------------------------------------------------------
      MatrixXd J = robot_model_->get_jacobian(q);
      MatrixXd Jdot = robot_model_->get_jacobian_dot(q, dq);

      Matrix4d T = robot_model_->get_fk_solver().forward_kinematics(q);
      Vector3d pos = T.block<3, 1>(0, 3);
      Matrix3d R = T.topLeftCorner(3, 3);

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
      Vector3d eps_tilde = eta_e * eps_d - eta_d * eps_e - math::skew(eps_d) * eps_e;

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

      // Calculate Ko' for the orientation part of the stiffness matrix (see The Role of Euler Parameters In Robot Control Caccavale et al. 1999)
      Matrix3d E_tilde = eta_tilde * Matrix3d::Identity() - math::skew(eps_tilde);
      MatrixXd Kp_mark = Kp_;
      Kp_mark.bottomRightCorner<3, 3>() = 2.0 * E_tilde.transpose() * Kp_.bottomRightCorner<3, 3>();

      // Explicitly calculate the DLS pseudo-inverse, see eq. (3.59)
      Eigen::MatrixXd J_pinv = J.transpose() * (J * J.transpose() + kappaI).fullPivHouseholderQr().solve(Eigen::MatrixXd::Identity(6, 6));

      // Calculate the primary task command
      Eigen::VectorXd task_cmd = ddx_d + Kd_ * dx_tilde + Kp_mark * x_tilde - Jdot * dq;

      // Project primary task into joint space
      Eigen::VectorXd y_primary = J_pinv * task_cmd;

      // Calculate the null-space projection matrix: N = I - J_pinv * J_A

      const Eigen::Index n = robot_model_->get_dof();
      Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
      Eigen::MatrixXd N = I - J_pinv * J;

      // Secondary task (Joint Damping to stop drift)
      Eigen::VectorXd secondary_cmd = -Kd_null_ * dq; 

      // Combine primary and secondary tasks (see eq. 3.54)
      y_ = y_primary + N * secondary_cmd;
    }
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
