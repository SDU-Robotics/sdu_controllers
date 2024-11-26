#pragma once

#ifndef RNEA_HPP
#define RNEA_HPP

#include <memory>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::math
{
  class RecursiveNewtonEuler
  {
    public:
      explicit RecursiveNewtonEuler(std::shared_ptr<models::RobotModel> robot_model);

      ~RecursiveNewtonEuler() = default;

      Eigen::VectorXd inverse_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
        const Eigen::VectorXd &ddq, const Eigen::VectorXd &he);

    private:
      void forward(const Eigen::VectorXd &dq, const Eigen::VectorXd &ddq, const std::vector<Eigen::Matrix4d> T);

      void backward(const Eigen::VectorXd &he, const std::vector<Eigen::Matrix4d> T);

      std::shared_ptr<models::RobotModel> robot_model;

      Eigen::Matrix<double, 3, Eigen::Dynamic> omega, domega, ddp, ddpc, f, mu;
      Eigen::VectorXd tau;
      Eigen::Vector3d omega0, domega0, ddp0;
      Eigen::Matrix<double, Eigen::Dynamic, 3> CoM;
      std::vector<Eigen::Matrix3d> link_inertia;
  };
}

#endif //RNEA_HPP
