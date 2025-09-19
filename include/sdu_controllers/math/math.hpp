#pragma once
#ifndef SDU_CONTROLLERS_MATH_HPP
#define SDU_CONTROLLERS_MATH_HPP
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <memory>
#include <sdu_controllers/kinematics/dh_parameters.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::math
{
  /**
   * @brief Calculate the rotational velocity transformation.
   *
   *  The calculation is based on Eq. (3.64), from page 130, Robotics: Modelling, Planning and Control, Chapter 3.
   *  The analytical solution to the inverse is from Peter Corke's toolbox (Spatial Maths for Python). See
   *  specifically: https://github.com/bdaiinstitute/spatialmath-python/blob/master/symbolic/angvelxform.ipynb
   *
   * \f$ \mathbf{\tau} = \mathbf{B}(q)y + \mathbf{C}(q, \dot{q})\dot{q} + \mathbf{\tau}_{g} \f$
   *
   * @param Gamma Euler angles in ZYZ order.
   * @param inverse if the inverse of the transformation should be returned (calculated symbolically).
   * @returns rotation rate transformation matrix A
   */
  static Eigen::MatrixXd rot_vel_transform(Eigen::Vector3d Gamma, bool inverse = true)
  {
    using namespace std;
    double phi = Gamma[0];
    double theta = Gamma[1];
    double psi = Gamma[2];

    Eigen::MatrixXd A(3, 3);
    if (!inverse)
    {
      // clang-format off
      A << 0, -sin(phi), cos(phi)*sin(theta),
           0, cos(phi),  sin(phi)*sin(theta),
           1, 0,         cos(theta);
      // clang-format on
    }
    else
    {
      if (tan(theta) == 0 || sin(theta) == 0)
      {
        throw runtime_error("Math error: Attempted to divide by zero\n");
      }
      // analytical inverse
      // clang-format off
      A << -cos(phi)/tan(theta), -sin(phi)/tan(theta), 1,
           -sin(phi),            cos(phi),             0,
           cos(phi)/sin(theta),  sin(phi)/sin(theta),  0;
      // clang-format on
    }
    return A;
  }

  static Eigen::MatrixXd jacobian_analytical(
      const Eigen::VectorXd &q,
      const std::shared_ptr<models::RobotModel> &robot_model)
  {
    using namespace Eigen;
    Matrix4d T = kinematics::forward_kinematics(q, robot_model);
    Matrix3d rot_mat = T.topLeftCorner(3, 3);
    Vector3d Gamma = rot_mat.eulerAngles(2, 1, 2);  // ZYZ representation
    Matrix3d A_inv = rot_vel_transform(Gamma);
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
    Matrix4d T = kinematics::forward_kinematics(q, robot_model);
    Matrix3d rot_mat = T.topLeftCorner(3, 3);
    Vector3d Gamma = rot_mat.eulerAngles(2, 1, 2);  // ZYZ representation
    Matrix3d A_inv = rot_vel_transform(Gamma);
    MatrixXd A_full = MatrixXd::Zero(6, 6);
    A_full.setIdentity();
    A_full.block<3, 3>(3, 3) = A_inv;
    MatrixXd Jdot_A = A_full * robot_model->get_jacobian_dot(q, dq);
    return Jdot_A;
  }

  static Eigen::Matrix<double, 6, Eigen::Dynamic> geometric_jacobian(
      const std::vector<Eigen::Matrix4d>& T_chain,
      const std::vector<kinematics::DHParam>& dh_parameters)
  {
    Eigen::Vector3d z_im1, o_im1, o_n;
    Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, dh_parameters.size());

    // Get position of end effector
    o_n = T_chain.back().template block<3, 1>(0, 3);

    // Loop through joints
    for (int i = 0; i < dh_parameters.size(); i++)
    {
      Eigen::Matrix4d T = T_chain[i];

      // Get z_{i-1}, o_{i-1}
      // z_{i-1} is the unit vector along the z-axis of the previous joint
      // o_{i-1} is the position of the center of the previous joint frame
      if (i > 0)
      {
        z_im1 = T_chain[i - 1].block<3, 1>(0, 2);
        o_im1 = T_chain[i - 1].block<3, 1>(0, 3);
      }
      else
      {
        // Base Position
        z_im1 = Eigen::Vector3d(0, 0, 1);
        o_im1 = Eigen::Vector3d(0, 0, 0);
      }

      // Assign the appropriate blocks of the Jacobian
      // These formulas are from Spong.
      if (dh_parameters[i].is_joint_revolute)
      {
        // Revolute joint
        J.template block<3, 1>(0, i) = z_im1.cross(o_n - o_im1);
        J.template block<3, 1>(3, i) = z_im1;
      }
      else
      {
        // Prismatic joint
        J.template block<3, 1>(0, i) = z_im1;
        J.template block<3, 1>(3, i).fill(0);
      }
    }

    return J;
  }

  static Eigen::Matrix<double, 6, Eigen::Dynamic> geometric_jacobian(
      Eigen::VectorXd q,
      const std::vector<kinematics::DHParam>& dh_parameters)
  {
    // Get the forward kinematics chain
    std::vector<Eigen::Matrix4d> T_chain = kinematics::forward_kinematics_all(q, dh_parameters);
    // Calculate the Jacobian
    return geometric_jacobian(T_chain, dh_parameters);
  }

  template<class Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> &vec)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3)

    // clang-format off
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() <<
            0.0, -vec[2], vec[1],
            vec[2], 0.0, -vec[0],
            -vec[1], vec[0], 0.0)
        .finished();
  }
  // clang-format on

  static Eigen::MatrixXd adjoint(const Eigen::Affine3d &T)
  {
    Eigen::MatrixXd adj = Eigen::MatrixXd::Zero(6, 6);
    adj.block<3, 3>(0, 0) = T.linear();
    adj.block<3, 3>(3, 0) = sdu_controllers::math::skew(T.translation()) * T.linear();
    adj.block<3, 3>(3, 3) = T.linear();
    return adj;
  }

  static Eigen::VectorXd
  wrench_trans(const Eigen::Vector3d &torques, const Eigen::Vector3d &forces, const Eigen::Affine3d &T)
  {
    Eigen::VectorXd wrench_in_A(6);
    wrench_in_A << torques[0], torques[1], torques[2], forces[0], forces[1], forces[2];
    Eigen::VectorXd wrench_in_B = adjoint(T).transpose() * wrench_in_A;
    return wrench_in_B;
  }

  static Eigen::Quaterniond exp(const Eigen::Quaterniond &quat)
  {
    using ::std::cos;
    using ::std::sin;

    double theta = quat.vec().norm();
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    Eigen::Quaterniond q;

    if (theta > static_cast<double>(0))
    {
      q.vec() = sin_theta * quat.vec() / theta;
    }
    else
    {
      q.vec().setZero();
    }

    q.w() = cos_theta;

    return q;
  }
}  // namespace sdu_controllers::math

// namespace sdu_controllers::math

#endif  // SDU_CONTROLLERS_MATH_HPP
