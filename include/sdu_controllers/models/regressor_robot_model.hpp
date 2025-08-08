#ifndef SDU_CONTROLLERS_MODELS_REGRESSOR_ROBOT_MODEL_HPP
#define SDU_CONTROLLERS_MODELS_REGRESSOR_ROBOT_MODEL_HPP

#include <Eigen/Core>
#include <sdu_controllers/kinematics/dh_parameters.hpp>
#include <sdu_controllers/models/robot_model.hpp>
#include <utility>  // for std::pair
#include <vector>

namespace sdu_controllers::models
{
  class RegressorRobotModel : public sdu_controllers::models::RobotModel
  {
   public:
    RegressorRobotModel(
        std::vector<kinematics::DHParam> dh_parameters,
        const Eigen::Vector3d& g0 = Eigen::Vector3d(0, 0, -9.81));

    /**
     * @brief Get inertia matrix \f$ \mathbf{B}(q) \f$
     * @param q the robot joint configuration.
     * @returns the inertia matrix.
     */
    virtual Eigen::MatrixXd get_inertia_matrix(const Eigen::VectorXd& q);

    /**
     * @brief Get coriolis matrix \f$ \mathbf{C}(q, \dot{q}) \f$
     * @param q the robot joint configuration.
     * @param qd the robot joint configuration.
     * @returns the coriolis matrix.
     */
    virtual Eigen::MatrixXd get_coriolis(const Eigen::VectorXd& q, const Eigen::VectorXd& qd);

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
    virtual Eigen::MatrixXd get_jacobian(const Eigen::VectorXd& q);

    /**
     * @brief Get jacobian dot \f$ \mathbf{\dot{J(q, dq)}} \f$
     * @returns the jacobian dot
     */
    virtual Eigen::MatrixXd get_jacobian_dot(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

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

    virtual uint16_t get_dof() const;

    virtual std::vector<double> get_a();

    virtual std::vector<double> get_d();

    virtual std::vector<double> get_alpha();

    virtual std::vector<double> get_theta();

    virtual std::vector<double> get_m();

    virtual std::vector<bool> get_is_joint_revolute();

    virtual Eigen::Vector3d get_g0();

    virtual Eigen::Matrix<double, Eigen::Dynamic, 3> get_CoM();

    virtual std::vector<Eigen::Matrix3d> get_link_inertia();

    virtual Eigen::MatrixXd get_regressor(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd)
        const = 0;

    virtual Eigen::MatrixXd get_friction_regressor(const Eigen::VectorXd& qd) const = 0;
    virtual Eigen::MatrixXd get_friction_regressor(const std::vector<double>& qd) const;

    virtual Eigen::VectorXd get_parameters() const = 0;

    virtual Eigen::VectorXd get_friction_parameters() const = 0;

    virtual Eigen::VectorXd get_tau(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd);

    std::vector<kinematics::DHParam> get_dh_parameters() const
    {
      return _dh_parameters;
    }

   protected:
    std::vector<kinematics::DHParam> _dh_parameters;
    Eigen::Vector3d _gravity;  // gravity vector
  };
}  // namespace sdu_controllers::models

#endif  // JIGGLYBOT_DYNAMIC_MODEL_HPP