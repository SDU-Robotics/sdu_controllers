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
   * @returns the gravity vector
   */
  Eigen::MatrixXd get_gravity(const Eigen::VectorXd &q) override;

  /**
 * @brief Get joint position bounds.
 * @returns the joint position bounds
 */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> get_joint_pos_bounds() const override;

  /**
   * @brief Get joint velocity bounds.
   * @returns the joint velocity bounds
   */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> get_joint_vel_bounds() const override;

  /**
   * @brief Get joint acceleration bounds.
   * @returns the joint acceleration bounds
   */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> get_joint_acc_bounds() const override;

  /**
   * @brief Get joint torque bounds.
   * @returns the joint torque bounds
   */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> get_joint_torque_bounds() const override;

  uint16_t get_dof() const override;

private:
  uint16_t dof_{ROBOT_DOF};
  std::pair<Eigen::VectorXd, Eigen::VectorXd> joint_pos_bounds_;
  std::pair<Eigen::VectorXd, Eigen::VectorXd> joint_vel_bounds_;
  std::pair<Eigen::VectorXd, Eigen::VectorXd> joint_acc_bounds_;
  std::pair<Eigen::VectorXd, Eigen::VectorXd> joint_torque_bounds_;
  URRobot ur_robot_;

};

} // namespace sdu_controllers::models

#endif  // SDU_CONTROLLERS_UR_ROBOT_MODEL_HPP
