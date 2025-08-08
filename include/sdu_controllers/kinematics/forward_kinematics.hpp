#pragma once
#ifndef SDU_CONTROLLERS_FORWARD_KINEMATICS_HPP
#define SDU_CONTROLLERS_FORWARD_KINEMATICS_HPP

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <sdu_controllers/kinematics/dh_parameters.hpp>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::kinematics
{
  static Eigen::Matrix4d forward_kinematics(const Eigen::VectorXd& q, const std::shared_ptr<models::RobotModel>& robot_model)
  {
    bool first = true;
    Eigen::Matrix4d T;

    std::vector<double> a = robot_model->get_a();
    std::vector<double> d = robot_model->get_d();
    std::vector<double> alpha = robot_model->get_alpha();
    std::vector<double> theta = robot_model->get_theta();
    std::vector<bool> is_joint_revolute = robot_model->get_is_joint_revolute();

    Eigen::MatrixXd A_i(4, 4);
    double qi;

    // calculate transform matrix for each link
    for (Eigen::Index i = 0; i < q.size(); i++)
    {
      if (is_joint_revolute.at(i))
      {  // revolute joint
        qi = q[i] + theta[i];

        A_i(0, 0) = std::cos(qi);
        A_i(0, 1) = -std::sin(qi) * std::cos(alpha[i]);
        A_i(0, 2) = std::sin(qi) * std::sin(alpha[i]);
        A_i(0, 3) = a[i] * std::cos(qi);

        A_i(1, 0) = std::sin(qi);
        A_i(1, 1) = std::cos(qi) * std::cos(alpha[i]);
        A_i(1, 2) = -std::cos(qi) * std::sin(alpha[i]);
        A_i(1, 3) = a[i] * std::sin(qi);

        A_i(2, 0) = 0;
        A_i(2, 1) = std::sin(alpha[i]);
        A_i(2, 2) = std::cos(alpha[i]);
        A_i(2, 3) = d[i];

        A_i(3, 0) = 0;
        A_i(3, 1) = 0;
        A_i(3, 2) = 0;
        A_i(3, 3) = 1;
      }
      else
      {  // prismatic joint
        qi = q[i] + d[i];
        A_i(0, 0) = std::cos(theta[i]);
        A_i(0, 1) = -std::sin(theta[i]) * std::cos(alpha[i]);
        A_i(0, 2) = std::sin(theta[i]) * std::sin(alpha[i]);
        A_i(0, 3) = a[i] * std::cos(theta[i]);

        A_i(1, 0) = std::sin(theta[i]);
        A_i(1, 1) = std::cos(theta[i]) * std::cos(alpha[i]);
        A_i(1, 2) = -std::cos(theta[i]) * std::sin(alpha[i]);
        A_i(1, 3) = a[i] * std::sin(theta[i]);

        A_i(2, 0) = 0;
        A_i(2, 1) = std::sin(alpha[i]);
        A_i(2, 2) = std::cos(alpha[i]);
        A_i(2, 3) = qi;

        A_i(3, 0) = 0;
        A_i(3, 1) = 0;
        A_i(3, 2) = 0;
        A_i(3, 3) = 1;
      }

      if (first)
      {
        T = A_i;
        first = false;
      }
      else
        T *= A_i;
    }

    return T;
  }

  static std::vector<Eigen::Matrix4d> forward_kinematics_all(
      const Eigen::VectorXd& q,
      const std::shared_ptr<models::RobotModel>& robot_model)
  {
    std::vector<Eigen::Matrix4d> complete_T;

    bool first = true;
    Eigen::Matrix4d T;

    std::vector<double> a = robot_model->get_a();
    std::vector<double> d = robot_model->get_d();
    std::vector<double> alpha = robot_model->get_alpha();
    std::vector<double> theta = robot_model->get_theta();
    std::vector<bool> is_joint_revolute = robot_model->get_is_joint_revolute();

    Eigen::MatrixXd A_i(4, 4);
    double qi;

    // calculate transform matrix for each link
    for (Eigen::Index i = 0; i < q.size(); i++)
    {
      double cos_alpha = std::cos(alpha[i]);
      double sin_alpha = std::sin(alpha[i]);
      if (is_joint_revolute.at(i))
      {  // revolute joint
        qi = q[i] + theta[i];

        double cos_qi = std::cos(qi);
        double sin_qi = std::sin(qi);

        A_i(0, 0) = cos_qi;
        A_i(0, 1) = -sin_qi * cos_alpha;
        A_i(0, 2) = sin_qi * sin_alpha;
        A_i(0, 3) = a[i] * cos_qi;

        A_i(1, 0) = sin_qi;
        A_i(1, 1) = cos_qi * cos_alpha;
        A_i(1, 2) = -cos_qi * sin_alpha;
        A_i(1, 3) = a[i] * sin_qi;

        A_i(2, 0) = 0;
        A_i(2, 1) = sin_alpha;
        A_i(2, 2) = cos_alpha;
        A_i(2, 3) = d[i];

        A_i(3, 0) = 0;
        A_i(3, 1) = 0;
        A_i(3, 2) = 0;
        A_i(3, 3) = 1;
      }
      else
      {  // prismatic joint

        double cos_theta = std::cos(theta[i]);
        double sin_theta = std::sin(theta[i]);

        qi = q[i] + d[i];
        A_i(0, 0) = cos_theta;
        A_i(0, 1) = -sin_theta * cos_alpha;
        A_i(0, 2) = sin_theta * sin_alpha;
        A_i(0, 3) = a[i] * cos_theta;

        A_i(1, 0) = sin_theta;
        A_i(1, 1) = cos_theta * cos_alpha;
        A_i(1, 2) = -cos_theta * sin_alpha;
        A_i(1, 3) = a[i] * sin_theta;

        A_i(2, 0) = 0;
        A_i(2, 1) = sin_alpha;
        A_i(2, 2) = cos_alpha;
        A_i(2, 3) = qi;

        A_i(3, 0) = 0;
        A_i(3, 1) = 0;
        A_i(3, 2) = 0;
        A_i(3, 3) = 1;
      }

      if (first)
      {
        T = A_i;
        first = false;
      }
      else
        T *= A_i;

      complete_T.push_back(T);
    }

    return complete_T;
  }

  std::vector<Eigen::Matrix4d> forward_kinematics_all(const Eigen::VectorXd& q, const std::vector<DHParam>& dh_parameters);

}  // namespace sdu_controllers::kinematics

#endif  // SDU_CONTROLLERS_FORWARD_KINEMATICS_HPP
