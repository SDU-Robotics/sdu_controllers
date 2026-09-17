#pragma once
#ifndef SDU_CONTROLLERS_KINEMATICS_JACOBIAN_ANALYTICAL_HPP
#define SDU_CONTROLLERS_KINEMATICS_JACOBIAN_ANALYTICAL_HPP
#include <Eigen/Dense>
#include <memory>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/math/math.hpp>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::kinematics
{
  static Eigen::MatrixXd jacobian_analytical(
      const Eigen::VectorXd &q,
      const std::shared_ptr<models::RobotModel> &robot_model)
  {
    using namespace Eigen;
    Matrix4d T = robot_model->get_fk_solver().forward_kinematics(q);
    Matrix3d rot_mat = T.topLeftCorner(3, 3);
    Vector3d Gamma = rot_mat.eulerAngles(2, 1, 2);  // ZYZ representation
    Matrix3d A_inv = math::rot_vel_transform(Gamma);
    MatrixXd A_full = MatrixXd::Zero(6, 6);
    A_full.setIdentity();
    A_full.block<3, 3>(3, 3) = A_inv;
    MatrixXd J_A = A_full * robot_model->get_jacobian(q);
    return J_A;
  }

  static Eigen::MatrixXd jacobian_dot_analytical(
      const Eigen::VectorXd &q,
      const Eigen::VectorXd &dq,
      const std::shared_ptr<models::RobotModel> &robot_model)
  {
    using namespace Eigen;
    Matrix4d T = robot_model->get_fk_solver().forward_kinematics(q);
    Matrix3d rot_mat = T.topLeftCorner(3, 3);
    Vector3d Gamma = rot_mat.eulerAngles(2, 1, 2);  // ZYZ representation
    Matrix3d A_inv = math::rot_vel_transform(Gamma);
    MatrixXd A_full = MatrixXd::Zero(6, 6);
    A_full.setIdentity();
    A_full.block<3, 3>(3, 3) = A_inv;
    MatrixXd Jdot_A = A_full * robot_model->get_jacobian_dot(q, dq);
    return Jdot_A;
  }
}  // namespace sdu_controllers::kinematics

#endif  // SDU_CONTROLLERS_KINEMATICS_JACOBIAN_ANALYTICAL_HPP
