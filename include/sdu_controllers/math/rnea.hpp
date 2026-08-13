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
      explicit RecursiveNewtonEuler(models::RobotModel &robot_model);

      ~RecursiveNewtonEuler() = default;

      Eigen::VectorXd inverse_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
        const Eigen::VectorXd &ddq, const Eigen::VectorXd &he);

      Eigen::VectorXd forward_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
        const Eigen::VectorXd &tau);

      void set_z0(const Eigen::Vector3d &z0);

      Eigen::MatrixXd inertia(const Eigen::VectorXd &q);
      Eigen::MatrixXd coriolis(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
      Eigen::VectorXd velocity_product(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
      Eigen::VectorXd gravity(const Eigen::VectorXd &q);
      
      // Get complete joint dynamics of every frame component
      const Eigen::Matrix3Xd& get_joint_forces() const { return f_; }
      const Eigen::Matrix3Xd& get_joint_moments() const { return mu_; }

      /**
       * @brief Classical joint friction torque \f$ \tau_{f} \f$.
       *
       * Computes \f$ \tau_{f} = \mathbf{F}_{v}\dot{q} + \mathbf{F}_{s}\,
       * \mathrm{sgn}(\dot{q}) \f$
       *
       * @param dq robot joint velocities.
       * @returns the friction torque vector.
       */
      Eigen::VectorXd friction(const Eigen::VectorXd &dq);

    private:
      /**
       * @brief Rigid-body inverse dynamics (without joint friction).
       */
      Eigen::VectorXd rigid_body_inverse_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
        const Eigen::VectorXd &ddq, const Eigen::VectorXd &he);

      void forward(const Eigen::VectorXd &dq, const Eigen::VectorXd &ddq, const std::vector<Eigen::Matrix4d> T);

      void backward(const Eigen::VectorXd &he, const std::vector<Eigen::Matrix4d> T);

      models::RobotModel &robot_model_;

      Eigen::Matrix<double, 3, Eigen::Dynamic> omega_, domega_, ddp_, ddpc_, f_, mu_;
      Eigen::VectorXd tau_;
      Eigen::Vector3d omega0_, domega0_, ddp0_, z0_;
      Eigen::Matrix<double, Eigen::Dynamic, 3> CoM_;
      std::vector<Eigen::Matrix3d> link_inertia_;
  };

} // namespace sdu_controllers::math

#endif // SDU_CONTROLLERS_RNEA_HPP
