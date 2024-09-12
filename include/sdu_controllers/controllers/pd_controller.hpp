#pragma once
#ifndef SDU_CONTROLLERS_PD_CONTROLLER_HPP
#define SDU_CONTROLLERS_PD_CONTROLLER_HPP

#include <Eigen/Dense>
#include <sdu_controllers/controllers/controller.hpp>

namespace sdu_controllers::controllers
{
  class PDController : public Controller
  {
   public:
    explicit PDController();
    ~PDController() override;

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
  };
}


#endif  // SDU_CONTROLLERS_PD_CONTROLLER_HPP
