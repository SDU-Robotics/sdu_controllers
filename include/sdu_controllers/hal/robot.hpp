#pragma once
#ifndef SDU_CONTROLLERS_ROBOT_HPP
#define SDU_CONTROLLERS_ROBOT_HPP

namespace sdu_controllers::hal
{
  /**
   * This class provides a Robot base class for the different robot interfaces
   * implemented in the (HAL) Hardware Abstraction Layer of sdu_controllers.
   */

  class Robot
  {
   public:
    virtual ~Robot() = default;
  };

}  // namespace sdu_controllers::hal

#endif  // SDU_CONTROLLERS_ROBOT_HPP
