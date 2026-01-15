#include <iostream>
#include <sdu_controllers/models/parameter_robot_model.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>
#include <sdu_controllers/utils/utility.hpp>

constexpr double pi = 3.14159265358979323846;
using namespace Eigen;

namespace sdu_controllers::models
{
  URRobotModel::URRobotModel(RobotType robot_type)
      : URRobotModel(
            robot_type == RobotType::ur3e   ? utils::ConfigFolder::find_config_file("ur3e_robot.yaml")
            : robot_type == RobotType::ur5e ? utils::ConfigFolder::find_config_file("ur5e_robot.yaml")
                                            : utils::ConfigFolder::find_config_file("ur10e_robot.yaml"))
  {
  }

  URRobotModel::URRobotModel(const std::string &yaml_filepath) : ParameterRobotModel(yaml_filepath)
  {
  }

  URRobotModel::URRobotModel(const std::filesystem::path &yaml_filepath) : ParameterRobotModel(yaml_filepath.string())
  {
  }

  URRobotModel::URRobotModel(const RobotParameters &params) : ParameterRobotModel(params)
  {
  }

}  // namespace sdu_controllers::models
