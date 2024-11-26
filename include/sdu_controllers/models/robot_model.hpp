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
     * @returns the gravity vector
     */
    virtual Eigen::MatrixXd get_gravity(const Eigen::VectorXd &q) = 0;

    /**
     * @brief Get the jacobian \f$ \mathbf{J(q)} \f$
     * @returns the jacobian
     */
    virtual Eigen::MatrixXd get_jacobian(const Eigen::VectorXd &q) = 0;

    /**
     * @brief Get jacobian dot \f$ \mathbf{\dot{J(q, dq)}} \f$
     * @returns the jacobian dot
     */
    virtual Eigen::MatrixXd get_jacobian_dot(const Eigen::VectorXd &q, const Eigen::VectorXd &dq) = 0;

    /**
     * @brief Get joint position bounds.
     * @returns the joint position bounds
     */
    virtual std::pair<Eigen::VectorXd, Eigen::VectorXd> get_joint_pos_bounds() = 0;

    /**
     * @brief Get joint velocity bounds.
     * @returns the joint velocity bounds
     */
    virtual std::pair<Eigen::VectorXd, Eigen::VectorXd> get_joint_vel_bounds() = 0;

    /**
     * @brief Get joint acceleration bounds.
     * @returns the joint acceleration bounds
     */
    virtual std::pair<Eigen::VectorXd, Eigen::VectorXd> get_joint_acc_bounds() = 0;

    /**
     * @brief Get joint torque bounds.
     * @returns the joint torque bounds
     */
    virtual std::pair<Eigen::VectorXd, Eigen::VectorXd> get_joint_torque_bounds() = 0;


    virtual uint16_t get_dof() const = 0;

    virtual std::vector<double> get_a() = 0;

    virtual std::vector<double> get_d() = 0;

    virtual std::vector<double> get_alpha() = 0;

    virtual std::vector<double> get_theta() = 0;

    virtual std::vector<bool> get_is_joint_revolute() = 0;

    virtual Eigen::Vector3d get_g0() = 0;

    virtual Eigen::Matrix<double, Eigen::Dynamic, 3> get_CoM() = 0;

    virtual std::vector<Eigen::Matrix3d> get_link_inertia() = 0;

  };

}  // namespace sdu_controllers::models

#endif  // SDU_CONTROLLERS_ROBOT_MODEL_HPP
