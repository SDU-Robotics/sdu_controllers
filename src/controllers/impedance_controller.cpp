#include <sdu_controllers/controllers/impedance_controller.hpp>
#include <sdu_controllers/math/math.hpp>
#include <utility>

using namespace Eigen;

namespace sdu_controllers::controllers
{
  ImpedanceController::ImpedanceController(
      MatrixXd Kp,
      MatrixXd Kd,
      MatrixXd Md,
      std::shared_ptr<models::RobotModel> robot_model,
      OrientationRepresentation orientation_rep)
      : Kp_(std::move(Kp)),
        Kd_(std::move(Kd)),
        Md_(std::move(Md)),
        robot_model_(std::move(robot_model)),
        orientation_rep_(orientation_rep)
  {
  }

  void ImpedanceController::step(
      const VectorXd &x_d,
      const VectorXd &dx_d,
      const VectorXd &ddx_d,
      const VectorXd &q,
      const VectorXd &dq,
      const VectorXd &h_d_e,
      const Vector4d &quat_d)
  {
    if (orientation_rep_ == OrientationRepresentation::ZYZ)
    {
      // ---------------------------------------------------------------
      // ZYZ path - analytical Jacobian, Euler-angle orientation error
      // ---------------------------------------------------------------
      Matrix4d T = robot_model_->get_fk_solver().forward_kinematics(q);
      Vector3d pos = T.block<3, 1>(0, 3);
      Matrix3d rot_mat = T.topLeftCorner(3, 3);
      Vector3d rot_zyz = rot_mat.eulerAngles(2, 1, 2);

      VectorXd x_e(6);
      x_e << pos, rot_zyz;

      MatrixXd J_A    = math::jacobian_analytical(q, robot_model_);
      MatrixXd Jdot_A = math::jacobian_dot_analytical(q, dq, robot_model_);

      VectorXd x_tilde  = x_d - x_e;
      VectorXd dx_tilde = dx_d - J_A * dq;

      VectorXd rhs = Kd_ * dx_tilde + Kp_ * x_tilde
                     - Md_ * Jdot_A * dq + Md_ * ddx_d - h_d_e;
      y_ = J_A.lu().solve(Md_.lu().solve(rhs));
    }
    else
    {
      // ---------------------------------------------------------------
      // QUATERNION path - geometric Jacobian, quaternion orientation error
      // ---------------------------------------------------------------
      // Geometric Jacobian [linear vel (3); angular vel (3)]
      MatrixXd J    = robot_model_->get_jacobian(q);
      MatrixXd Jdot = robot_model_->get_jacobian_dot(q, dq);

      // Current end-effector pose
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

      // Quaternion orientation error
      Vector3d eps_tilde = eta_e * eps_d - eta_d * eps_e
                           - math::skew(eps_d) * eps_e;

      // Position error
      Vector3d p_tilde = x_d.head<3>() - pos;

      VectorXd x_tilde(6);
      x_tilde << p_tilde, eps_tilde;

      // Velocity error
      VectorXd v_e = J * dq;
      VectorXd dx_tilde(6);
      dx_tilde << (dx_d.head<3>() - v_e.head<3>()),
                  (dx_d.tail<3>() - v_e.tail<3>());

      VectorXd rhs = Kd_ * dx_tilde + Kp_ * x_tilde
                     - Md_ * Jdot * dq + Md_ * ddx_d - h_d_e;
      y_ = J.lu().solve(Md_.lu().solve(rhs));
    }
  }

  void ImpedanceController::reset()
  {
    Kp_.setZero();
    Kd_.setZero();
    Md_.setZero();
    y_.setZero();
  }

  VectorXd ImpedanceController::get_output()
  {
    return y_;
  }

}  // namespace sdu_controllers::controllers
