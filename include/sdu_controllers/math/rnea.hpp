#pragma once
#ifndef SDU_CONTROLLERS_RNEA_HPP
#define SDU_CONTROLLERS_RNEA_HPP

#include <memory>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::math
{
   /**
   *  An implementation of the recursive Newton-Euler algorithm
   */
  class RecursiveNewtonEuler
  {
    public:
      explicit RecursiveNewtonEuler(std::shared_ptr<models::RobotModel> robot_model);

      ~RecursiveNewtonEuler() = default;

      Eigen::VectorXd inverse_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
        const Eigen::VectorXd &ddq, const Eigen::VectorXd &he);

      Eigen::VectorXd forward_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
        const Eigen::VectorXd &tau, const Eigen::VectorXd &he);

      void set_z0(const Eigen::Vector3d &z0);

      Eigen::MatrixXd inertia(const Eigen::VectorXd &q);
      Eigen::VectorXd velocity_product(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
      Eigen::VectorXd gravity(const Eigen::VectorXd &q);

    private:
      void forward(const Eigen::VectorXd &dq, const Eigen::VectorXd &ddq, const std::vector<Eigen::Matrix4d> T);

      void backward(const Eigen::VectorXd &he, const std::vector<Eigen::Matrix4d> T);

      std::shared_ptr<models::RobotModel> robot_model;

      Eigen::Matrix<double, 3, Eigen::Dynamic> omega, domega, ddp, ddpc, f, mu;
      Eigen::VectorXd tau;
      Eigen::Vector3d omega0, domega0, ddp0, z0;
      Eigen::Matrix<double, Eigen::Dynamic, 3> CoM;
      std::vector<Eigen::Matrix3d> link_inertia;
  };

} // namespace sdu_controllers::math

#endif // SDU_CONTROLLERS_RNEA_HPP
