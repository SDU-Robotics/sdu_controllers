#pragma once
#ifndef SDU_CONTROLLERS_ROBOT_MODEL_HPP
#define SDU_CONTROLLERS_ROBOT_MODEL_HPP

#include <Eigen/Dense>
namespace sdu_controllers::kinematics
{
  class ForwardKinematics;
}

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
     * @brief Calculate the inverse dynamics.
     * @param q robot joint positions.
     * @param dq robot joint velocities.
     * @param ddq robot joint accelerations.
     * @param he end-effector wrench.
     * @returns the computed torques for the joint actuators \f$ \tau \f$
     */
    virtual Eigen::VectorXd inverse_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
      const Eigen::VectorXd &ddq, const Eigen::VectorXd &he) = 0;


    /**
     * @brief Calculate the forward dynamics.
     * @param q robot joint positions.
     * @param dq robot joint velocities.
     * @param tau joint torques of the robot
     * @returns the acceleration \f$ \ddot{q} \f$
     */
    virtual Eigen::VectorXd forward_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &tau) = 0;

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
     * @brief Get maximum joint velocity.
     * @returns the maximum joint velocity
     */
    virtual Eigen::VectorXd get_joint_max_vel() = 0;

    /**
     * @brief Get maximum joint acceleration.
     * @returns the maximum joint acceleration
     */
    virtual Eigen::VectorXd get_joint_max_acc() = 0;

    /**
     * @brief Get maximum joint torque.
     * @returns the maximum joint torque
     */
    virtual Eigen::VectorXd get_joint_max_torque() = 0;

    /**
     * @brief Get the degrees of freedom of the robot.
     * @returns the number of degrees of freedom
     */
    virtual uint16_t get_dof() const = 0;

    /**
     * @brief Get the masses of the robot links.
     * @returns vector containing the mass of each link
     */
    virtual std::vector<double> get_m() = 0;
    
    /**
     * @brief Get the gravity vector in base frame.
     * @returns the 3D gravity vector
     */
    virtual Eigen::Vector3d get_g0() = 0;

    /**
     * @brief Get the center of mass positions for each link.
     * @returns matrix where each row contains the 3D center of mass position for a link
     */
    virtual Eigen::Matrix<double, Eigen::Dynamic, 3> get_CoM() = 0;

    /**
     * @brief Get the inertia tensors for each link.
     * @returns vector of 3x3 inertia matrices for each link
     */
    virtual std::vector<Eigen::Matrix3d> get_link_inertia() = 0;

    /**
     * @brief Get the forward kinematics solver instance.
     * @returns reference to the forward kinematics solver
     */
    virtual const kinematics::ForwardKinematics &get_fk_solver() const = 0;
  };

}  // namespace sdu_controllers::models

#endif  // SDU_CONTROLLERS_ROBOT_MODEL_HPP
