#pragma once
#include <stdexcept>
#ifndef SDU_CONTROLLERS_PARAMETER_ROBOT_MODEL_HPP
#define SDU_CONTROLLERS_PARAMETER_ROBOT_MODEL_HPP

#include <sdu_controllers/models/robot_parameters.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/math/rnea.hpp>
#include <sdu_controllers/models/robot_parameters.hpp>
#include <Eigen/Dense>
#include <optional>
#include <vector>
#include <string>

namespace sdu_controllers::models
{
  /**
   * Base class that offers helper to load robot parameter sets from YAML.
   */

  class ParameterRobotModel : public RobotModel
  {
   public:
    explicit ParameterRobotModel() = default;
    explicit ParameterRobotModel(const std::string &yaml_filepath);
    explicit ParameterRobotModel(const RobotParameters &params);
    virtual ~ParameterRobotModel() = default;

    /**
     * @brief Load robot parameters from YAML file
     * @param filepath the filepath to the YAML file containing the robot parameters.
     * @returns returns empty optional on parse/validation failure.
     */
    static std::optional<RobotParameters> load_parameters_from_yaml(const std::string &filepath);

    void set_robot_parameters(const RobotParameters &params);

    /**
     * @brief Calculate the inverse dynamics.
     *
     * Uses the recursive newton euler algorithm. (TODO: add reference to book)
     *
     * @param y auxiliary control input.
     * @param q robot joint positions.
     * @param dq robot joint velocities.
     * @returns the computed torques for the joint actuators \f$ \tau \f$
     */
    Eigen::VectorXd inverse_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
      const Eigen::VectorXd &ddq, const Eigen::VectorXd &he) override;


    /**
     * @brief Calculate the forward dynamics.
     *
     * Uses the recursive newton euler algorithm. (TODO: add reference to book)
     *
     * @param q robot joint positions.
     * @param dq robot joint velocities.
     * @param tau joint torques of the robot
     * @returns the acceleration \f$ \ddot{q} \f$
     */
    Eigen::VectorXd forward_dynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &tau) override;

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
     * @brief Get the jacobian \f$ \mathbf{J(q)} \f$
     * @returns the jacobian
     */
    Eigen::MatrixXd get_jacobian(const Eigen::VectorXd &q) override;

    /**
     * @brief Get jacobian dot \f$ \mathbf{\dot{J(q, dq)}} \f$
     * @returns the jacobian dot
     */
    Eigen::MatrixXd get_jacobian_dot(const Eigen::VectorXd &q, const Eigen::VectorXd &dq) override;

    /**
     * @brief Get joint position bounds.
     * @returns the joint position bounds
     */
    std::pair<Eigen::VectorXd, Eigen::VectorXd> get_joint_pos_bounds() override;

    /**
     * @brief Get maximum joint velocity.
     * @returns the maximum joint velocity
     */
    Eigen::VectorXd get_joint_max_vel() override;

    /**
     * @brief Get maximum joint acceleration.
     * @returns the maximum joint acceleration
     */
    Eigen::VectorXd get_joint_max_acc() override;

    /**
     * @brief Get maximum joint torque.
     * @returns the maximum joint torque
     */
    Eigen::VectorXd get_joint_max_torque() override;

    uint16_t get_dof() const override;

    std::vector<double> get_m() override;

    std::vector<bool> get_is_joint_revolute() override;

    Eigen::Vector3d get_g0() override;

    Eigen::Matrix<double, Eigen::Dynamic, 3> get_CoM() override;

    std::vector<Eigen::Matrix3d> get_link_inertia() override;

    const kinematics::ForwardKinematics &get_fk_solver() const override;

    void set_com(const Eigen::Matrix<double, Eigen::Dynamic, 3> &com);

    void set_link_inertia(const std::vector<Eigen::Matrix3d> &link_inertia);

    void set_mass(const std::vector<double> &mass);

  protected:
    // degrees of freedom
    uint16_t dof_;

    // forward kinematics model (either based on dh parameters or frames)
    std::shared_ptr<kinematics::ForwardKinematics> fk_model_;

    // recursive newton euler algorithm
    std::shared_ptr<math::RecursiveNewtonEuler> rnea_;

    // inertial
    std::vector<double> mass_;
    Eigen::Matrix<double, Eigen::Dynamic, 3> com_;
    std::vector<Eigen::Matrix3d> link_inertia_;
    std::vector<bool> is_joint_revolute_;
    Eigen::Vector3d g0_;

    // joint limits
    std::pair<Eigen::VectorXd, Eigen::VectorXd> joint_pos_bounds_;
    Eigen::VectorXd joint_max_velocity_;
    Eigen::VectorXd joint_max_acceleration_;
    Eigen::VectorXd joint_max_torque_;
  };

}  // namespace sdu_controllers::models

#endif  // SDU_CONTROLLERS_ROBOT_MODEL_HPP
