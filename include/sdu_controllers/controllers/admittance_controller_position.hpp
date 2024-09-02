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
     * @brief Set the input force as a 3-dimensional vector \f$[x\, y\, z]\f$.
     */
    void set_input_force(const Eigen::Vector3d &force);

    /**
     * @brief Set the input torque. as a 3-dimensional vector \f$[\tau_{x}\, \tau_{y}\, \tau_{z}]\f$.
     */
    void set_input_torque(const Eigen::Vector3d &torque);

    /**
     * @brief Set the input position as a 3-dimensional vector \f$[x\, y\, z]\f$.
     */
    void set_input_position(const Eigen::Vector3d &position);

    /**
     * @brief Set the input orientation as a quaternion \f$[w\, x\, y\, z]\f$ (scalar-first).
     */
    void set_input_orientation(const Eigen::Quaterniond &orientation);

    /**
     * @brief Step the execution of the controller.
     */
    void step() override;

    /**
     * @brief Get the state of the controller. Updates when the step() function is called.
     */
    void get_state() override;

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
  };
}  // namespace sdu_controllers::controllers

#endif  // SDU_CONTROLLERS_ADMITTANCE_CONTROLLER_POSITION_HPP
