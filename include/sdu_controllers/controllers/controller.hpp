#pragma once
#ifndef SDU_CONTROLLERS_CONTROLLER_HPP
#define SDU_CONTROLLERS_CONTROLLER_HPP

namespace sdu_controllers::controllers
{
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
