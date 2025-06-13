#include <sdu_controllers/controllers/operational_space_controller.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/math/math.hpp>
#include <utility>

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
  }

  void OperationalSpaceController::step(
      const VectorXd &x_d,
      const VectorXd &dx_d,
      const VectorXd &ddx_d,
      const VectorXd &q,
      const VectorXd &dq)
  {
    Matrix4d T = kinematics::forward_kinematics(q, robot_model_);
    VectorXd pos = T.block<3, 1>(0,3);
    // std::cout << "pos: " << pos << std::endl;
    Matrix3d rot_mat = T.topLeftCorner(3, 3);
    // std::cout << "rot_mat: " << rot_mat << std::endl;
    VectorXd rot_zyz = rot_mat.eulerAngles(2, 1, 2); // ZYZ representation
    // std::cout << "rot_zyz: " << rot_zyz << std::endl;
    VectorXd x_e(pos.size() + rot_zyz.size());
    x_e << pos, rot_zyz;
    // std::cout << "x_e: " << x_e << std::endl;
    VectorXd x_tilde = x_d - x_e;
    // std::cout << "x_tilde: " << x_tilde << std::endl;
    MatrixXd J_A = math::jacobian_analytical(q, robot_model_);
    // std::cout << "J_A: " << J_A << std::endl;
    MatrixXd Jdot_A = math::jacobian_dot_analytical(q, dq, robot_model_);
    // std::cout << "Jdot_A: " << Jdot_A << std::endl;
    VectorXd dx_e = J_A * dq;
    // std::cout << "dx_e: " << dx_e << std::endl;
    VectorXd dx_tilde = dx_d - dx_e;
    // std::cout << "dx_tilde: " << dx_tilde << std::endl;

    // Eq. (8.114) from page 348, Robotics: Modelling, Planning and Control:
    // TODO: This will only work for a robot with six joints, since you cannot take the inverse
    //       of a 6x7 sized Jacobian.
    y_ = J_A.lu().solve(ddx_d + Kd_ * dx_tilde + Kp_ * x_tilde - Jdot_A * dq);
  }

  void OperationalSpaceController::reset()
  {
    Kp_.setZero();
    Kd_.setZero();
    y_.setZero();
  }

  VectorXd OperationalSpaceController::get_output()
  {
    return y_;
  }

}  // namespace sdu_controllers::controllers
