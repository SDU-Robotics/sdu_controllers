#ifndef SDU_CONTROLLERS_MODELS_REGRESSOR_ROBOT_MODEL_HPP
#define SDU_CONTROLLERS_MODELS_REGRESSOR_ROBOT_MODEL_HPP

#include <Eigen/Core>
#include <memory>
#include <sdu_controllers/kinematics/dh_kinematics.hpp>
#include <sdu_controllers/models/robot_model.hpp>
#include <utility>
#include <vector>

namespace sdu_controllers::models
{
  /**
   * @brief Regressor-based robot model for dynamics computation.
   * This class implements a robot model using regressor matrices to compute dynamics.
   * It inherits from the RobotModel base class and provides implementations for inverse and forward dynamics,
   * as well as methods to retrieve inertia, coriolis, gravity, jacobian, and jacobian dot matrices.
   *
   * This class is an abstract base class and requires derived classes to implement methods for obtaining
   * the regressor matrices and friction regressor matrices, as well as other robot-specific parameters, such as joint
   * limits.
   */
  class RegressorRobotModel : public sdu_controllers::models::RobotModel
  {
   public:
    /**
     * @brief Construct a new Regressor Robot Model.
     *
     * @param fkModel shared pointer to forward kinematics solver
     * @param g0 gravity vector in base frame
     * @throws std::invalid_argument if fkModel is nullptr
     */
    RegressorRobotModel(
        const std::shared_ptr<kinematics::ForwardKinematics>& fkModel,
        const Eigen::Vector3d& g0 = Eigen::Vector3d(0, 0, -9.81));

    /**
     * @brief Calculate the inverse dynamics.
     * @param q robot joint positions.
     * @param dq robot joint velocities.
     * @param ddq robot joint accelerations.
     * @param he end-effector wrench.
     * @returns the computed torques for the joint actuators \f$ \tau \f$
     */
    virtual Eigen::VectorXd inverse_dynamics(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& dq,
        const Eigen::VectorXd& ddq,
        const Eigen::VectorXd& he) override;

    /**
     * @brief Calculate the forward dynamics.
     * @param q robot joint positions.
     * @param dq robot joint velocities.
     * @param tau joint torques of the robot
     * @returns the acceleration \f$ \ddot{q} \f$
     */
    virtual Eigen::VectorXd forward_dynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& tau)
        override;

    /**
     * @brief Get inertia matrix \f$ \mathbf{B}(q) \f$
     * @param q the robot joint configuration.
     * @returns the inertia matrix.
     */
    virtual Eigen::MatrixXd get_inertia_matrix(const Eigen::VectorXd& q) override;

    /**
     * @brief Get coriolis matrix \f$ \mathbf{C}(q, \dot{q}) \f$
     * @param q the robot joint configuration.
     * @param qd the robot joint configuration.
     * @returns the coriolis matrix.
     */
    virtual Eigen::MatrixXd get_coriolis(const Eigen::VectorXd& q, const Eigen::VectorXd& qd) override;

    /**
     * @brief Get gravity term \f$ \tau_{g} \f$
     * @returns the gravity vector
     */
    virtual Eigen::MatrixXd get_gravity(const Eigen::VectorXd& q) override;
    virtual Eigen::MatrixXd get_gravity(const std::vector<double>& q);

    /**
     * @brief Get the jacobian \f$ \mathbf{J(q)} \f$
     * @returns the jacobian
     */
    virtual Eigen::MatrixXd get_jacobian(const Eigen::VectorXd& q) override;

    /**
     * @brief Get jacobian dot \f$ \mathbf{\dot{J(q, dq)}} \f$
     * @returns the jacobian dot
     */
    virtual Eigen::MatrixXd get_jacobian_dot(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) override;

    /**
     * @brief Get the degrees of freedom of the robot.
     * @returns the number of degrees of freedom
     */
    virtual uint16_t get_dof() const override;

    /**
     * @brief Get the masses of the robot links.
     * @note Assumes that the masses are part of the dynamic parameters used in the regressor.
     * it will therefore just throw an error if called.
     * @throws std::runtime_error if called.
     * @returns vector containing the mass of each link
     */
    virtual std::vector<double> get_m() override;

    /**
     * @brief Get the gravity vector in base frame.
     * @returns the 3D gravity vector
     */
    virtual Eigen::Vector3d get_g0() override;

    /**
     * @brief Get the center of mass positions for each link.
     * @note Assumes that the CoM positions are part of the dynamic parameters used in the regressor.
     * it will therefore just throw an error if called.
     * @throws std::runtime_error if called.
     * @returns matrix where each row contains the 3D center of mass position for a link
     */
    virtual Eigen::Matrix<double, Eigen::Dynamic, 3> get_CoM() override;

    /**
     * @brief Get the inertia tensors for each link.
     * @note Assumes that the inertia tensors are part of the dynamic parameters used in the regressor.
     * it will therefore just throw an error if called.
     * @throws std::runtime_error if called.
     * @returns vector of 3x3 inertia matrices for each link
     */
    virtual std::vector<Eigen::Matrix3d> get_link_inertia() override;

    /**
     * @brief Get the friction regressor matrix \f$ \mathbf{Y_f(dq)} \f$
     * This function is alternative version taking std::vector<double> as input.
     * @see get_friction_regressor(const Eigen::VectorXd& qd)
     * @param qd [in] Joint velocities
     * @return The friction regressor matrix
     */
    virtual Eigen::MatrixXd get_friction_regressor(const std::vector<double>& qd) const;

    /**
     * @brief Get the forward kinematics solver instance.
     * @returns reference to the forward kinematics solver
     */
    virtual const kinematics::ForwardKinematics& get_fk_solver() const override;

    // ################################################
    // # Functions to be overridden by child classes  #
    // ################################################

    /**
     * @brief Get joint position bounds.
     * @returns the joint position bounds
     */
    virtual std::pair<Eigen::VectorXd, Eigen::VectorXd> get_joint_pos_bounds() override = 0;

    /**
     * @brief Get maximum joint velocity.
     * @returns the maximum joint velocity
     */
    virtual Eigen::VectorXd get_joint_max_vel() override = 0;

    /**
     * @brief Get maximum joint acceleration.
     * @returns the maximum joint acceleration
     */
    virtual Eigen::VectorXd get_joint_max_acc() override = 0;

    /**
     * @brief Get maximum joint torque.
     * @returns the maximum joint torque
     */
    virtual Eigen::VectorXd get_joint_max_torque() override = 0;


    /**
     * @brief Get the regressor matrix \f$ \mathbf{Y(q, \dot{q}, \ddot{q})} \f$
     * Use this function to compute the regressor matrix for given joint positions, velocities and accelerations.
     * Can be used in conjunction with the parameter vector to compute the inverse dynamics torques.
     * @param q [in] Joint positions
     * @param qd [in] Joint velocities
     * @param qdd [in] Joint accelerations
     * @return The regressor matrix
     */
    virtual Eigen::MatrixXd get_regressor(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd)
        const = 0;

    /**
     * @brief Get the friction regressor matrix \f$ \mathbf{Y_f(dq)} \f$
     * Use this function to compute the friction regressor matrix for given joint velocities.
     * Can be used in conjunction with the friction parameter vector to compute the friction torques.
     * @param qd [in] Joint velocities
     * @return The friction regressor matrix
     */
    virtual Eigen::MatrixXd get_friction_regressor(const Eigen::VectorXd& qd) const = 0;

    /**
     * @brief Get the dynamic parameters vector.
     * @returns the dynamic parameters vector
     */
    virtual Eigen::VectorXd get_parameters() const = 0;

    /**
     * @brief Get the friction parameters vector.
     * @returns the friction parameters vector
     */
    virtual Eigen::VectorXd get_friction_parameters() const = 0;

   protected:
    std::shared_ptr<kinematics::ForwardKinematics> fk_model_;
    Eigen::Vector3d gravity_;
  };
}  // namespace sdu_controllers::models

#endif  // JIGGLYBOT_DYNAMIC_MODEL_HPP
