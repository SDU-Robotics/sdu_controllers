#pragma once
#ifndef SDU_CONTROLLERS_INVERSE_DYNAMICS_HPP
#define SDU_CONTROLLERS_INVERSE_DYNAMICS_HPP

#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::math
{

  /**
   * This class provides a base class for the inverse dynamics
   * that can be calculated in joint-space or cartesian space, this
   * class contains the robot model defined by @see RobotModel class.
   */

  class InverseDynamics
  {
   public:
    explicit InverseDynamics(const models::RobotModel& robot_model) : robot_model_(robot_model)
    {
    }
    virtual ~InverseDynamics() = default;

   private:
    models::RobotModel robot_model_;
  };

}  // namespace sdu_controllers::math

#endif  // SDU_CONTROLLERS_INVERSE_DYNAMICS_HPP
