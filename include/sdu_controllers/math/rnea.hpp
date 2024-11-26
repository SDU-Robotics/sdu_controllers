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
      Eigen::VectorXd forward(const Eigen::VectorXd &dq, const Eigen::VectorXd &ddq,
        const std::vector<Eigen::Matrix4d> T);

      Eigen::VectorXd backward(const Eigen::VectorXd &omega, const Eigen::VectorXd &domega,
        const Eigen::VectorXd &he, const std::vector<Eigen::Matrix4d> T);
  };
}

#endif //RNEA_HPP
