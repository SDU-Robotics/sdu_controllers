#pragma once
#ifndef SDU_CONTROLLERS_IMPEDANCE_CONTROLLER_HPP
#define SDU_CONTROLLERS_IMPEDANCE_CONTROLLER_HPP

#include <Eigen/Dense>
#include <memory>
#include <sdu_controllers/controllers/controller.hpp>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::controllers
{
  /**
   * Impedance controller in operational space from
   * (Siciliano et al. Robotics: Modelling, Planning and Control, page 374)
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
   *
   * Control law (both modes share the same structure):
   * \f[
   *   \mathbf{y} = \mathbf{J}^{-1} \mathbf{M}^{-1}_{d}
   *     \left(
   *       \mathbf{K}_{D}\,\dot{\tilde{x}} + \mathbf{K}_{P}\,\tilde{x}
   *       - \mathbf{M}_{d}\,\dot{\mathbf{J}}\dot{\mathbf{q}}
   *       + \mathbf{M}_{d}\,\ddot{x}_{d}
   *       - \mathbf{h}^{d}_{e}
   *     \right)
   * \f]
   *
   * In ZYZ mode, \f$ \mathbf{J} \equiv \mathbf{J}_A \f$ and
   * \f$ \tilde{x} = [p_d - p_e;\; \Gamma_d - \Gamma_e] \f$.
   * In QUATERNION mode, \f$ \mathbf{J} \f$ is the geometric Jacobian and
   * \f$ \tilde{x} = [p_d - p_e;\; \tilde{\epsilon}] \f$ where
   * \f$ \tilde{\epsilon} = \eta_e\epsilon_d - \eta_d\epsilon_e - S(\epsilon_d)\epsilon_e \f$.
   */
  class ImpedanceController : public Controller
  {
   public:
    /** Orientation representation used for the error and Jacobian. */
    enum class OrientationRepresentation
    {
      ZYZ,        // ZYZ Euler angles + analytical Jacobian 
      QUATERNION  // Unit quaternion error + geometric Jacobian 
    };

    /**
     * @brief Construct an ImpedanceController.
     *
     * @param Kp               Cartesian stiffness matrix (6x6).
     * @param Kd               Cartesian damping matrix (6x6).
     * @param Md               Desired inertia matrix (6x6).
     * @param robot_model      Shared pointer to the robot model.
     * @param orientation_rep  Orientation representation (default: ZYZ).
     */
    explicit ImpedanceController(
        Eigen::MatrixXd Kp,
        Eigen::MatrixXd Kd,
        Eigen::MatrixXd Md,
        std::shared_ptr<models::RobotModel> robot_model,
        OrientationRepresentation orientation_rep = OrientationRepresentation::ZYZ);

    /**
     * @brief Compute one control step.
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
     * @param h_d_e  Desired contact wrench (6-vector; forces + torques).
     * @param quat_d Desired orientation quaternion [w, x, y, z] (QUATERNION mode only).
     */
    void step(
        const Eigen::VectorXd &x_d,
        const Eigen::VectorXd &dx_d,
        const Eigen::VectorXd &ddx_d,
        const Eigen::VectorXd &q,
        const Eigen::VectorXd &dq,
        const Eigen::VectorXd &h_d_e,
        const Eigen::Vector4d &quat_d = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0));

    /** @brief Get the joint-acceleration output (updated by step()). */
    Eigen::VectorXd get_output() override;

    /** @brief Reset internal controller state. */
    void reset() override;

   private:
    Eigen::VectorXd y_;
    Eigen::MatrixXd Kp_, Kd_, Md_;
    std::shared_ptr<models::RobotModel> robot_model_;
    OrientationRepresentation orientation_rep_;
  };

}  // namespace sdu_controllers::controllers

#endif  // SDU_CONTROLLERS_IMPEDANCE_CONTROLLER_HPP

