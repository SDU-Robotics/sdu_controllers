#pragma once
#ifndef SDU_CONTROLLERS_INVERSE_DYNAMICS_JOINT_SPACE_HPP
#define SDU_CONTROLLERS_INVERSE_DYNAMICS_JOINT_SPACE_HPP

#include <Eigen/Dense>
#include <sdu_controllers/math/inverse_dynamics.hpp>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::math
{

  /**
   * This class implements inverse dynamics calculation for joint-space.
   */

  class InverseDynamicsJointSpace : public InverseDynamics
  {
   public:
    explicit InverseDynamicsJointSpace(std::shared_ptr<models::RobotModel> robot_model);
    ~InverseDynamicsJointSpace() override;

    /**
     * @brief Calculate the inverse dynamics.
     *
     * The inverse dynamics are calculated according to Eq. (6.27),
     * from page 141, Springer Handbook of Robotics 2008:
     *
     * \f$ \mathbf{\tau} = \mathbf{B}(q)y + \mathbf{C}(q, \dot{q})\dot{q} + \mathbf{\tau}_{g} \f$
     *
     * @param y auxiliary control input.
     * @param q robot joint positions.
     * @param dq robot joint velocities.
     * @returns the computed torques for the joint actuators \f$ \tau \f$
     */
    [[nodiscard]] Eigen::VectorXd inverse_dynamics(
        const Eigen::VectorXd &y,
        const Eigen::VectorXd &q,
        const Eigen::VectorXd &dq) const;
  };

}  // namespace sdu_controllers::math

#endif  // SDU_CONTROLLERS_INVERSE_DYNAMICS_JOINT_SPACE_HPP
