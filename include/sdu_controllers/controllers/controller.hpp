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
     * @brief Step the execution of the controller (must be called in a loop externally).
     */
    virtual void step() = 0;

    /**
     * @brief Get the state of the controller. Updates when the step function is called.
     */
    virtual void get_state() = 0;

    /**
     * @brief Reset internal controller variables.
     */
    virtual void reset() = 0;

    virtual ~Controller() = default;
  };

}  // namespace sdu_controllers::controllers

#endif  // SDU_CONTROLLERS_CONTROLLER_HPP
