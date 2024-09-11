#pragma once
#ifndef SDU_CONTROLLERS_BLEEDING_BLANKET_ROBOT_MODEL_HPP
#define SDU_CONTROLLERS_BLEEDING_BLANKET_ROBOT_MODEL_HPP

#include <vector>
#include <Eigen/Dense>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::models
{

/**
 * This class provides implements a robot model for the EUROfusion breeding blanket handling robot.
 */

class BreedingBlanketHandlingRobotModel : public RobotModel
{
public:
  explicit BreedingBlanketHandlingRobotModel();

  ~BreedingBlanketHandlingRobotModel() override;

  /**
   * @brief Get inertia matrix \f$ \mathbf{B}(q) \f$
   * @param q the robot joint configuration.
   * @returns the inertia matrix.
   */
  Eigen::MatrixXd get_inertia_matrix(const Eigen::VectorXd &q) override;

  /**
   * @brief Get coriolis matrix \f$ \mathbf{C}(q, \dot{q}) \f$
   * @param q the robot joint configuration.
   * @param qd the robot joint configuration.
   * @returns the coriolis matrix.
   */
  Eigen::MatrixXd get_coriolis(const Eigen::VectorXd &q, const Eigen::VectorXd &qd) override;

  /**
   * @brief Get gravity term \f$ \tau_{g} \f$
   */
  Eigen::MatrixXd get_gravity(const Eigen::VectorXd &q) override;

};

} // namespace sdu_controllers::models

#endif  // SDU_CONTROLLERS_BLEEDING_BLANKET_ROBOT_MODEL_HPP
