#pragma once
#ifndef SDU_CONTROLLERS_FORWARD_KINEMATICS_HPP
#define SDU_CONTROLLERS_FORWARD_KINEMATICS_HPP

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::kinematics
{
  static Eigen::MatrixXd forward_kinematics(const Eigen::VectorXd& q, const std::shared_ptr<models::RobotModel>& robot_model)
  {
    bool first = true;
    Eigen::MatrixXd T(4,4);
    // calculate transform matrix for each link
    for (Eigen::Index i = 0; i < q.size(); i++)
    {
      std::vector<double> a = robot_model->get_a();
      std::vector<double> d = robot_model->get_d();
      std::vector<double> alpha = robot_model->get_alpha();

      Eigen::MatrixXd A_i(4, 4);
      A_i(0, 0) = std::cos(q[i]);
      A_i(0, 1) = -std::sin(q[i]) * std::cos(alpha[i]);
      A_i(0, 2) = std::sin(q[i]) * std::sin(alpha[i]);
      A_i(0, 3) = a[i] * std::cos(q[i]);

      A_i(1, 0) = std::sin(q[i]);
      A_i(1, 1) = std::cos(q[i]) * std::cos(alpha[i]);
      A_i(1, 2) = -std::cos(q[i]) * std::sin(alpha[i]);
      A_i(1, 3) = a[i] * std::sin(q[i]);

      A_i(2, 0) = 0;
      A_i(2, 1) = std::sin(alpha[i]);
      A_i(2, 2) = std::cos(alpha[i]);
      A_i(2, 3) = d[i];

      A_i(3, 0) = 0;
      A_i(3, 1) = 0;
      A_i(3, 2) = 0;
      A_i(3, 3) = 1;

      if (first)
      {
        T = A_i;
        first = false;
      }
      else
        T *= A_i;

    }
    // std::cout << "T: " << T << std::endl;
    return T;
  }
}  // namespace sdu_controllers::kinematics

#endif  // SDU_CONTROLLERS_FORWARD_KINEMATICS_HPP
