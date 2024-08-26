#pragma once
#ifndef SDU_CONTROLLERS_ADMITTANCE_CONTROLLER_POSITION_HPP
#define SDU_CONTROLLERS_ADMITTANCE_CONTROLLER_POSITION_HPP

#include <sdu_controllers/controllers/controller.hpp>

namespace sdu_controllers::controllers
{

/**
 * Admittance controller implementation as described in: Robotics Modelling, Planning and Control, Chapter 9, Eq. 9.37:
 *
 * \f$ \mathbf{M}_{t}\ddot{\tilde{z}} + \mathbf{K}_{Dt}\dot{\tilde{z}} + \mathbf{K}_{Pt}\tilde{z} = h_{e}^{d} \f$
 *
 * The output is a cartesian pose.
 */

class AdmittanceControllerPosition : public Controller
{
 public:
  explicit AdmittanceControllerPosition();
  ~AdmittanceControllerPosition() override;

  /**
   * \brief Step the execution of the controller.
   */
  void step() override;

  /**
   * \brief Get the state of the controller. Updates when the step() function is called.
   */
  void get_state() override;

  /**
   * \brief Reset internal controller variables.
   */
  void reset() override;
};

}  // namespace sdu_controllers::controllers

#endif  // SDU_CONTROLLERS_ADMITTANCE_CONTROLLER_POSITION_HPP
