#pragma once
#ifndef SDU_CONTROLLERS_MATH_HPP
#define SDU_CONTROLLERS_MATH_HPP
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <memory>
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

  static Eigen::MatrixXd jacobian_analytical(const Eigen::VectorXd &q, const std::shared_ptr<models::RobotModel>& robot_model)
  {
    using namespace Eigen;
    Matrix4d T = kinematics::forward_kinematics(q, robot_model);
    Matrix3d rot_mat = T.topLeftCorner(3, 3);
    Vector3d Gamma = rot_mat.eulerAngles(2, 1, 2); // ZYZ representation
    Matrix3d A_inv = rot_vel_transform(Gamma);
    MatrixXd A_full = MatrixXd::Zero(6, 6);
    A_full.setIdentity();
    A_full.block<3, 3>(3,3) = A_inv;
    MatrixXd J_A = A_full * robot_model->get_jacobian(q);
    return J_A;
  }

  static Eigen::MatrixXd jacobian_dot_analytical(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const std::shared_ptr<models::RobotModel>& robot_model)
  {
    using namespace Eigen;
    Matrix4d T = kinematics::forward_kinematics(q, robot_model);
    Matrix3d rot_mat = T.topLeftCorner(3, 3);
    Vector3d Gamma = rot_mat.eulerAngles(2, 1, 2); // ZYZ representation
    Matrix3d A_inv = rot_vel_transform(Gamma);
    MatrixXd A_full = MatrixXd::Zero(6, 6);
    A_full.setIdentity();
    A_full.block<3, 3>(3,3) = A_inv;
    std::cout << "Jdot:" << robot_model->get_jacobian_dot(q, dq) << std::endl;
    MatrixXd Jdot_A = A_full * robot_model->get_jacobian_dot(q, dq);
    return Jdot_A;
  }
}  // namespace sdu_controllers::math

#endif  // SDU_CONTROLLERS_MATH_HPP
