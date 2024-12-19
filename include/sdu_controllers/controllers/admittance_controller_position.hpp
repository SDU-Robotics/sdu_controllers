#pragma once
#ifndef SDU_CONTROLLERS_ADMITTANCE_CONTROLLER_POSITION_HPP
#define SDU_CONTROLLERS_ADMITTANCE_CONTROLLER_POSITION_HPP

#include <Eigen/Dense>
#include <sdu_controllers/controllers/controller.hpp>

namespace sdu_controllers::controllers
{
  /**
   * The admittance controller is implemented as described in: Robotics Modelling, Planning and Control, Chapter 9, Eq. 9.37:
   *
   * \f$ \mathbf{M}_{t}\ddot{\tilde{z}} + \mathbf{K}_{Dt}\dot{\tilde{z}} + \mathbf{K}_{Pt}\tilde{z} = h_{e}^{d} \f$
   *
   * The position and orientation are treated separately. For the position Eq. 9.37 is rearranged to solve for
   * the acceleration \f$\ddot{x}\f$, yielding:
   *
   * \f$ \ddot{x} = \mathbf{M}^{-1}f_{base}^{d} - \mathbf{K}x - \mathbf{D}\dot{x} \f$
   *
   * which is integrated twice to give a position.
   *
   * Similarly, for the orientation we rearrange Eq. 9.37, to solve for the angular acceleration \f$\ddot{\omega}\f$,
   * yielding:
   *
   * \f$ \ddot{\omega} = \mathbf{M}_{O}^{-1}\mu^{d} - \mathbf{K}^{'}_{O} \mathbf{q}_{e} - \mathbf{D}_{O}\dot{\omega} \f$
   *
   * The rotational stiffness \f$ \mathbf{K}^{'}_{O} \f$ is defined by Eq. 60 in F. Caccavale et al.: The Role of Euler
   * Parameters in Robot Control as:
   * \f$\mathbf{K}^{'}_{O} = 2\mathbf{E}^{T}\mathbf{K}_{O}\f$ with as \f$\mathbf{E} = \eta \mathbf{I} – \mathbf{S}(\epsilon) \f$.
   *
   * We then integrate the acceleration to calculate the angular velocity.
   * To integrate the velocity to the orientation in quaternions, we use
   * Eq. 61 in F. Caccavale et al.: The Role of Euler Parameters in Robot Control:
   * \f$ d\epsilon_{cd}^{d} = \frac{1}{2}\Delta\omega_{cd}^{d}dt \f$
   *
   *
   * The output of the controller is a cartesian pose with the orientation described as a quaternion (scalar-first).
   *
   * The admittance parameters default to:
   *
   * \f$\mathbf{M} = 22.5\f$, \f$\mathbf{M}_{O} = 0.25\f$
   *
   * \f$\mathbf{K} = 0\f$, \f$\mathbf{K}_{O} = 0\f$
   *
   * \f$\mathbf{D} = 70\f$, \f$\mathbf{D}_{O} = 3\f$
   */
  class AdmittanceControllerPosition : public Controller
  {
   public:
    /**
     * @brief
     * Initialize the admittance controller
     * @param frequency controller frequency, defaults to 500 Hz
     */
    explicit AdmittanceControllerPosition(double frequency=500.0);

    /**
     * @brief
     * Set the positional mass matrix \f$ \mathbf{M} \f$
     */
    void set_mass_matrix_position(const Eigen::Matrix3d &mass);

    /**
     * @brief
     * Set the positional stiffness matrix \f$ \mathbf{K} \f$
     */
    void set_stiffness_matrix_position(const Eigen::Matrix3d &stiffness);

    /**
     * @brief
     * Set the positional damping matrix \f$ \mathbf{D} \f$
     */
    void set_damping_matrix_position(const Eigen::Matrix3d &damping);

    /**
     * @brief
     * Set the orientational mass matrix \f$ \mathbf{M}_{O} \f$
     */
    void set_mass_matrix_orientation(const Eigen::Matrix3d &mass);

    /**
     * @brief
     * Set the orientational stiffness matrix \f$ \mathbf{K}_{O} \f$
     */
    void set_stiffness_matrix_orientation(const Eigen::Matrix3d &stiffness);

    /**
     * @brief
     * Set the orientational damping matrix \f$ \mathbf{D}_{O} \f$
     */
    void set_damping_matrix_orientation(const Eigen::Matrix3d &damping);

    /**
     * @brief Step the execution of the controller.
     *
     * @param input_force given as \f$ [f_{x}, f_{y}, f_{z}] \f$
     * @param input_torque given as \f$ [\mu_{x}, \mu_{y}, \mu_{z}] \f$
     * @param x_desired desired position given as \f$ [x, y, z] \f$
     * @param quat_desired desired orientation in quaternion \f$ [q_{w}, q_{x}, q_{y}, q_{z}]\f$ (scalar first)
     */
    void step(const Eigen::Vector3d &input_force, const Eigen::Vector3d &input_torque, const Eigen::Vector3d &x_desired, const Eigen::Vector4d &quat_desired);

    /**
     * @brief Get the output of the controller. Updates when the step() function is called.
     * @returns the new pose as \f$[x, y, z, q_{w}, q_{x}, q_{y}, q_{z}]\f$
     */
    Eigen::VectorXd get_output() override;

    /**
     * @brief Reset internal controller variables.
     */
    void reset() override;

   private:
    Eigen::Matrix3d M_;
    Eigen::Matrix3d K_;
    Eigen::Matrix3d D_;
    Eigen::Matrix3d Mo_;
    Eigen::Matrix3d Ko_;
    Eigen::Matrix3d Do_;

    // Control variables
    double dt_;
    Eigen::VectorXd u_{7};

    // Error terms position
    Eigen::Vector3d x_e_;
    Eigen::Vector3d dx_e_;

    // Error terms orientation
    Eigen::Quaterniond quat_e_;
    Eigen::Vector3d omega_e_;

    // Helper variables
    Eigen::Matrix3d rot_identity_;
  };
}  // namespace sdu_controllers::controllers

#endif  // SDU_CONTROLLERS_ADMITTANCE_CONTROLLER_POSITION_HPP
