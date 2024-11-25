
#include <sdu_controllers/controllers/force_control_inner_velocity_loop.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/math/math.hpp>
#include <utility>

using namespace Eigen;

namespace sdu_controllers::controllers
{
  // ForceControlInnerVelocityLoop::ForceControlInnerVelocityLoop(
  //   Eigen::MatrixXd Kp,
  //   Eigen::MatrixXd Kd,
  //   Eigen::MatrixXd Md,
  //   Eigen::MatrixXd Kf,
  //   std::shared_ptr<models::RobotModel> robot_model)

  ForceControlInnerVelocityLoop::ForceControlInnerVelocityLoop(
    Eigen::MatrixXd Kp,
    Eigen::MatrixXd Kd,
    Eigen::MatrixXd Md,
    Eigen::MatrixXd Kf,
    std::shared_ptr<models::RobotModel> robot_model)
      : Kp_(std::move(Kp)),
        Kd_(std::move(Kd)),
        Md_(std::move(Md)),
        Kf_(std::move(Kf)),
        robot_model_(std::move(robot_model))
  {
    Mdinv_ = Md.inverse();
  }

  void ForceControlInnerVelocityLoop::step(
    const Eigen::VectorXd &f_d,
    const Eigen::VectorXd &f_e,
    const Eigen::VectorXd &q,
    const Eigen::VectorXd &dq)
  {
    // Get current cartesian position
    Matrix4d T = kinematics::forward_kinematics(q, robot_model_);
    VectorXd pos = T.block<3, 1>(0,3);
    // Matrix3d rot_mat = T.topLeftCorner(3, 3);

    MatrixXd Jac = robot_model_->get_jacobian(q);
    std::cout << "Jac: " << Jac << std::endl;

    MatrixXd JacDot = robot_model_->get_jacobian_dot(q, dq);

    VectorXd vel = Jac * dq;

    VectorXd xf = Kf_ * (f_d - f_e);

    std::cout << "Xf: " << xf << std::endl;

    y_ = Jac.inverse() * (Mdinv_ * (-Kd_ * vel + Kp_ * xf - Md_ * JacDot * dq));

    std::cout << "y_: " << y_ << std::endl;
  }

  void ForceControlInnerVelocityLoop::reset()
  {
    Kp_.setZero();
    Kd_.setZero();
    Md_.setZero();
    Kf_.setZero();
    y_.setZero();
  }

  VectorXd ForceControlInnerVelocityLoop::get_output()
  {
    return y_;
  }
}