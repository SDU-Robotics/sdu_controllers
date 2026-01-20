#pragma once
#ifndef SDU_CONTROLLERS_BLEEDING_BLANKET_ROBOT_MODEL_HPP
#define SDU_CONTROLLERS_BLEEDING_BLANKET_ROBOT_MODEL_HPP

#include <Eigen/Dense>
#include <filesystem>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/math/rnea.hpp>
#include <sdu_controllers/models/robot_parameters.hpp>
#include <sdu_controllers/models/robot_model.hpp>
#include <sdu_controllers/models/parameter_robot_model.hpp>
#include <vector>

namespace sdu_controllers::models
{

  /**
   * This class provides a robot model for the EUROfusion breeding blanket handling robot.
   */

  class BreedingBlanketHandlingRobotModel : public ParameterRobotModel
  {
   public:
    BreedingBlanketHandlingRobotModel();
    explicit BreedingBlanketHandlingRobotModel(const std::string &yaml_filepath);
    explicit BreedingBlanketHandlingRobotModel(const std::filesystem::path &yaml_filepath);
    explicit BreedingBlanketHandlingRobotModel(const RobotParameters &params);
    virtual ~BreedingBlanketHandlingRobotModel() = default;

    void set_tcp_mass(double mass, Eigen::Vector3d& com, Eigen::Matrix3d inertia);

   private:
    std::vector<double> mass_default_;
    Eigen::Matrix<double, 7, 3> com_default_;
    std::vector<Eigen::Matrix3d> link_inertia_default_;
  };

}  // namespace sdu_controllers::models

#endif  // SDU_CONTROLLERS_BLEEDING_BLANKET_ROBOT_MODEL_HPP
