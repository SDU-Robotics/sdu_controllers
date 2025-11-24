#pragma once
#ifndef SDU_CONTROLLERS_UR_ROBOT_MODEL_HPP
#define SDU_CONTROLLERS_UR_ROBOT_MODEL_HPP

#include <Eigen/Dense>
#include <memory>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/math/rnea.hpp>
#include <sdu_controllers/models/robot_parameters.hpp>
#include <sdu_controllers/models/robot_model.hpp>
#include <sdu_controllers/models/parameter_robot_model.hpp>


#include <vector>

namespace sdu_controllers::models
{
  /**
   * This class provides a robot model for a UR robot.
   */
  class URRobotModel : public ParameterRobotModel
  {
   public:

    enum RobotType
    {
      ur3e,
      ur5e,
      ur10e
    };

    explicit URRobotModel(RobotType robot_type);
    explicit URRobotModel(const std::string &yaml_filepath);
    explicit URRobotModel(const RobotParameters &params);
    virtual ~URRobotModel() = default;
  };

}  // namespace sdu_controllers::models

#endif  // SDU_CONTROLLERS_UR_ROBOT_MODEL_HPP
