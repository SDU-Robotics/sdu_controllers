#pragma once
#ifndef SDU_CONTROLLERS_FORWARD_DYNAMICS_HPP
#define SDU_CONTROLLERS_FORWARD_DYNAMICS_HPP
#include <algorithm>
#include <memory>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::math
{
  /**
   * This class implements forward dynamics computing the acceleration due to applied torque. It
   * requires a robot model defined by @see RobotModel class.
   */

  class ForwardDynamics
  {
   public:
    explicit ForwardDynamics(std::shared_ptr<models::RobotModel> robot_model, double dt);

    ~ForwardDynamics() = default;

    /**
     * @brief Calculate the forward dynamics.
     *
     * The forward dynamics are calculated according to Eq. (7.115), from page 293,
     * Robotics: Modelling, Planning and Control, Chapter 7:
     *
     * \f$ \ddot{q} = \mathbf{B}^{-1}(q) \left(\tau - \mathbf{C}(q)\dot{q} -\mathbf{\tau}_{g}\right) \f$
     *
     * @param q robot joint positions.
     * @param dq robot joint velocities.
     * @param tau joint torques of the robot
     * @returns the acceleration \f$ \ddot{q} \f$
     */
    Eigen::VectorXd forward_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &tau);

    std::shared_ptr<models::RobotModel> get_robot_model()
    {
      return robot_model_;
    }

   private:
    double dt_;
    std::shared_ptr<models::RobotModel> robot_model_;
  };

}  // namespace sdu_controllers::math

#endif  // SDU_CONTROLLERS_FORWARD_DYNAMICS_HPP
