#pragma once
#ifndef SDU_CONTROLLERS_ROBOT_MODEL_HPP
#define SDU_CONTROLLERS_ROBOT_MODEL_HPP

#include <Eigen/Dense>

namespace sdu_controllers::models
{
  /**
   * This class provides a base class for the RobotModel.
   */

  class RobotModel
  {
   public:
    explicit RobotModel() = default;

    virtual ~RobotModel() = default;

    /**
     * @brief Get inertia matrix \f$ \mathbf{B}(q) \f$
     * @param q the robot joint configuration.
     * @returns the inertia matrix.
     */
    virtual Eigen::MatrixXd get_inertia_matrix(const Eigen::VectorXd &q) = 0;

    /**
     * @brief Get coriolis matrix \f$ \mathbf{C}(q, \dot{q}) \f$
     * @param q the robot joint configuration.
     * @param qd the robot joint configuration.
     * @returns the coriolis matrix.
     */
    virtual Eigen::MatrixXd get_coriolis(const Eigen::VectorXd &q, const Eigen::VectorXd &qd) = 0;

    /**
     * @brief Get gravity term \f$ \tau_{g} \f$
     */
    virtual Eigen::MatrixXd get_gravity(const Eigen::VectorXd &q) = 0;
  };

}  // namespace sdu_controllers::models

#endif  // SDU_CONTROLLERS_ROBOT_MODEL_HPP
