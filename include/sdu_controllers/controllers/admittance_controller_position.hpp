#pragma once
#ifndef SDU_CONTROLLERS_ADMITTANCE_CONTROLLER_POSITION_HPP
#define SDU_CONTROLLERS_ADMITTANCE_CONTROLLER_POSITION_HPP

#include <Eigen/Dense>
#include <sdu_controllers/controllers/controller.hpp>

namespace sdu_controllers::controllers
{
  /**
   * Admittance controller implementation as described in: Robotics Modelling, Planning and Control, Chapter 9, Eq. 9.37:
   *
   * \f$ \mathbf{M}_{t}\ddot{\tilde{z}} + \mathbf{K}_{Dt}\dot{\tilde{z}} + \mathbf{K}_{Pt}\tilde{z} = h_{e}^{d} \f$
   *
   * The position and orientation are treated separately, for the position Eq. 9.37 is rearranged to solve for
   * the acceleration \f$\ddot{x}\f$, yielding:
   *
   * \f$ \ddot{x} = \mathbf{M}^{-1}f_{base}^{d} - \mathbf{K}x - \mathbf{D}\dot{x} \f$
   *
   * which is integrated twice to give a position.
   *
   * Similarly for the orientation we rearrange Eq. 9.37, to solve for the angular acceleration \f$\ddot{\omega}\f$,
   * yielding:
   *
   * \f$ \ddot{\omega} = \mathbf{M}_{O}^{-1}\mu^{d} - \mathbf{K}^{'}_{O} \mathbf{q}_{e} - \mathbf{D}_{O}\dot{\omega} \f$
   *
   * where \f$\mathbf{K}^{'}_{O} = 2\mathbf{E}^{T}\mathbf{K}_{O}\f$.
   *
   * Eq. 61 in F. Caccavale et al.: The Role of Euler Parameters in Robot Control is:
   *
   * \f$ d\epsilon_{cd}^{d} = \frac{1}{2}\Delta\omega_{cd}^{d}dt \f$
   *
   * TODO: finish description of rotational part of the controller.
   *
   * The output of the controller is a cartesian pose.
   */
  class AdmittanceControllerPosition : public Controller
  {
   public:
    explicit AdmittanceControllerPosition();
    ~AdmittanceControllerPosition() override;

    /**
     * @brief
     * Set the positional mass matrix
     */
    void set_mass_matrix_position(const Eigen::Matrix3d &mass);

    /**
     * @brief
     * Set the positional stiffness matrix
     */
    void set_stiffness_matrix_position(const Eigen::Matrix3d &stiffness);

    /**
     * @brief
     * Set the positional damping matrix
     */
    void set_damping_matrix_position(const Eigen::Matrix3d &damping);

    /**
     * @brief
     * Set the orientational mass matrix
     */
    void set_mass_matrix_orientation(const Eigen::Matrix3d &mass);

    /**
     * @brief
     * Set the orientational stiffness matrix
     */
    void set_stiffness_matrix_orientation(const Eigen::Matrix3d &stiffness);

    /**
     * @brief
     * Set the orientational damping matrix
     */
    void set_damping_matrix_orientation(const Eigen::Matrix3d &damping);


    /**
     * @brief
     * Set the time interval betweens steps (used for integration, -> 1./control frequency)
     */
    void set_time_interval(double dt);

    /**
     * @brief Step the execution of the controller.
     */
    void step(const Eigen::Vector3d &input_force, const Eigen::Vector3d &input_torque, const Eigen::Vector3d &x_desired, const Eigen::Quaterniond &quat_desired);

    /**
     * @brief Get the output of the controller. Updates when the step() function is called.
     * Returns the new pose as \f$[x, y, z, q_{w}, q_{x}, q_{y}, q_{z}]\f$
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
