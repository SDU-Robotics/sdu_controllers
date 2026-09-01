#pragma once
#ifndef SDU_CONTROLLERS_CONTROLLER_HPP
#define SDU_CONTROLLERS_CONTROLLER_HPP

namespace sdu_controllers::controllers
{
  /**
   * Orientation representation used by controllers that support both ZYZ Euler-angle
   * and unit-quaternion orientation errors.
   */
  enum class OrientationRepresentation
  {
    ZYZ,        // ZYZ Euler angles + analytical Jacobian (singular at theta=0, pi)
    QUATERNION  // Unit quaternion error + geometric Jacobian (singularity-free)
  };

  /**
   * This class provides a base class for the different controllers. This is useful when
   * dealing with multiple controllers.
   */

  class Controller
  {
   public:
    /**
     * @brief Get the output of the controller. Should be updated when the controller step function is called.
     */
    virtual Eigen::VectorXd get_output() = 0;

    /**
     * @brief Reset internal controller variables.
     */
    virtual void reset() = 0;

    virtual ~Controller() = default;
  };

}  // namespace sdu_controllers::controllers

#endif  // SDU_CONTROLLERS_CONTROLLER_HPP
