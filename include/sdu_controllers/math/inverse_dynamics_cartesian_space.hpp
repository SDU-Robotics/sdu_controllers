#pragma once
#ifndef SDU_CONTROLLERS_INVERSE_DYNAMICS_CARTESIAN_SPACE_HPP
#define SDU_CONTROLLERS_INVERSE_DYNAMICS_CARTESIAN_SPACE_HPP

#include <Eigen/Dense>
#include <sdu_controllers/math/inverse_dynamics.hpp>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::math
{

  /**
   * This class implements inverse dynamics in cartesian space.
   */

  class InverseDynamicsCartesianSpace : public InverseDynamics
  {
   public:
    explicit InverseDynamicsCartesianSpace(std::shared_ptr<models::RobotModel> robot_model);

    /**
     * @brief Calculate inverse dynamics in cartesian space.
     *
     * The inverse dynamics are calculated according to Eq. (7.133) and Eq. (7.141).
     * from page 297-299, Robotics: Modelling, Planning and Control.
     *
     * First we calculate \f$ \gamma \f$ which expresses the contribution of the end-effector forces due to joint
     * actuation Eq. (7.133):
     *
     * \f$ \gamma_{A} = \mathbf{B}_{A}(x_{e})\ddot{x}_{e} + \mathbf{C}_{A}(x_{e}, \dot{x}_{e})\dot{x}_{e} + \mathbf{g}_{A}(x_{e}) \f$
     *
     * where
     *
     * \f$ \mathbf{B}_{A} = \mathbf{J}^{-T}_{A}\mathbf{B}\mathbf{J}^{-1}_{A} \f$
     *
     * \f$ \mathbf{C}_{A}\dot{x}_{e} = \mathbf{J}^{-T}_{A}\mathbf{C}\dot{q}-\mathbf{B}_{A}\dot{\mathbf{J}}\dot{q} \f$
     *
     * \f$ \mathbf{g}_{A} = \mathbf{J}^{-T}_{A}\mathbf{g} \f$
     *
     * Then we can calculate the torques according to Eq. (7.141):
     *
     * \f$ \mathbf{\tau} = \mathbf{J}^{T}_{A}(q)\gamma + \mathbf{I}_{n}-\mathbf{J}^{T}_{A}(q)\bar{\mathbf{J}}^{T}_{A}(q) \f$
     *
     * @param y auxiliary control input.
     * @param x_ee robot end-effector position.
     * @param dx_ee robot end-effector velocity.
     * @returns the computed torques for the joint actuators \f$ \tau \f$
     */
    Eigen::VectorXd inverse_dynamics(
        const Eigen::VectorXd &y,
        const Eigen::VectorXd &x_ee,
        const Eigen::VectorXd &dx_ee,
        const Eigen::VectorXd &ddx_ee);
  };

}  // namespace sdu_controllers::math

#endif  // SDU_CONTROLLERS_INVERSE_DYNAMICS_CARTESIAN_SPACE_HPP
