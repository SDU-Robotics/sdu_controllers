#pragma once
#ifndef SDU_CONTROLLERS_UR_ROBOT_MODEL_HPP
#define SDU_CONTROLLERS_UR_ROBOT_MODEL_HPP

#include <Eigen/Dense>
#include <memory>
#include <sdu_controllers/models/robot_model.hpp>
#include <sdu_controllers/models/ur_robot.hpp>
#include <vector>

constexpr uint16_t ROBOT_DOF = 6;

namespace sdu_controllers::models
{

/**
 * This class provides a robot model for a UR robot.
 */

class URRobotModel : public RobotModel
{
public:
  URRobotModel(RobotType robot_type);

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

  uint16_t get_dof() const;

private:
  uint16_t dof_{ROBOT_DOF};
  URRobot ur_robot_;

};

} // namespace sdu_controllers::models

#endif  // SDU_CONTROLLERS_UR_ROBOT_MODEL_HPP
