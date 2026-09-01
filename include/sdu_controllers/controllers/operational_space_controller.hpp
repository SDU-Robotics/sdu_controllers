#pragma once
#ifndef SDU_CONTROLLERS_OPERATIONAL_SPACE_CONTROLLER_HPP
#define SDU_CONTROLLERS_OPERATIONAL_SPACE_CONTROLLER_HPP

#include <Eigen/Dense>
#include <memory>
#include <sdu_controllers/controllers/controller.hpp>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::controllers
{
  /**
   * Operational (or cartesian) space controller with inverse dynamics control. The
   * controller is implemented according to Eq. (8.114) from page 348, Robotics: Modelling, Planning and Control:
   *
   * \f$ \mathbf{y} = \mathbf{J}_{A}^{-1}(q)\left(\ddot{x}_{d} + \mathbf{K}_{D}\dot{\tilde{x}} +
   * \mathbf{K}_{P}\tilde{x}-\mathbf{\dot{J}}_{A}(q, \dot{q})\dot{q}\right) \f$
   *
   * Two orientation representations are supported, selectable at construction time:
   *
   * **ZYZ** (default) - uses the analytical Jacobian \f$ \mathbf{J}_A \f$ and
   * expresses the orientation error as a ZYZ Euler-angle difference.
   * Singular at \f$ \theta = 0 \f$ and \f$ \theta = \pi \f$.
   *
   * **QUATERNION** - uses the geometric Jacobian \f$ \mathbf{J} \f$ and
   * expresses the orientation error via the unit-quaternion error vector,
   * which is singularity-free.
   */
  class OperationalSpaceController : public Controller
  {
   public:
    /** Orientation representation used for the error and Jacobian. */
    using OrientationRepresentation = controllers::OrientationRepresentation;

    /**
     * @brief Construct an OperationalSpaceController.
     *
     * @param Kp               Cartesian stiffness/proportional gain matrix (6x6).
     * @param Kd               Cartesian damping/derivative gain matrix (6x6).
     * @param robot_model      Shared pointer to the robot model.
     * @param orientation_rep  Orientation representation (default: ZYZ).
     */
    explicit
    OperationalSpaceController(
        Eigen::MatrixXd Kp,
        Eigen::MatrixXd Kd,
        std::shared_ptr<models::RobotModel> robot_model,
        OrientationRepresentation orientation_rep = OrientationRepresentation::ZYZ);

    /**
     * @brief Step the execution of the controller.
     *
     * In **ZYZ** mode, \p x_d must be a 6-vector [position(3), ZYZ angles(3)];
     * \p quat_d is ignored.
     *
     * In **QUATERNION** mode, only \p x_d[0:3] (position) is used for the pose
     * part; \p quat_d [w, x, y, z] provides the desired orientation.
     *
     * @param x_d    Desired pose   (6-vector; see above).
     * @param dx_d   Desired Cartesian velocity (6-vector).
     * @param ddx_d  Desired Cartesian acceleration (6-vector).
     * @param q      Current joint positions.
     * @param dq     Current joint velocities.
     * @param quat_d Desired orientation quaternion [w, x, y, z] (QUATERNION mode only).
     */
    void step(
        const Eigen::VectorXd &x_d,
        const Eigen::VectorXd &dx_d,
        const Eigen::VectorXd &ddx_d,
        const Eigen::VectorXd &q,
        const Eigen::VectorXd &dq,
        const Eigen::Vector4d &quat_d = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0));

    /**
     * @brief Get the output of the controller. Updates when the step() function is called.
     */
    Eigen::VectorXd get_output() override;

    /**
     * @brief Reset internal controller variables.
     */
    void reset() override;

    /**
     * Set kappa used in the damped least squares.
     */
    void set_kappa(double kappa);

  private:
    Eigen::VectorXd y_;
    Eigen::MatrixXd Kp_, Kd_;
    double kappa_;
    std::shared_ptr<models::RobotModel> robot_model_;
    OrientationRepresentation orientation_rep_;
  };

}  // namespace sdu_controllers::controllers

#endif  // SDU_CONTROLLERS_OPERATIONAL_SPACE_CONTROLLER_HPP
