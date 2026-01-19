#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <optional>
#include <sdu_controllers/models/robot_parameters.hpp>
#include <sdu_controllers/models/parameter_robot_model.hpp>
#include <string>

using namespace sdu_controllers::models;

static std::string source_path(const std::string &rel) {
#ifdef PROJECT_SOURCE_DIR
  return std::string(PROJECT_SOURCE_DIR) + "/" + rel;
#else
  return rel;
#endif
}

TEST_CASE("load_parameters_from_yaml: UR5e loads correctly", "[yaml][ur5e]")
{
  const std::string path = source_path("config/models/ur5e_robot.yaml");
  auto params_opt = ParameterRobotModel::load_parameters_from_yaml(path);
  REQUIRE(params_opt.has_value());

  const RobotParameters &p = *params_opt;

  // basic checks
  REQUIRE(p.dof == 6u);
  REQUIRE(p.mass.size() == p.dof);
  REQUIRE(p.com.rows() == static_cast<int>(p.dof));
  REQUIRE(p.com.cols() == 3);
  REQUIRE(p.joint_position_bounds.second.size() == static_cast<int>(p.dof));
}

TEST_CASE("load_parameters_from_yaml: UR3e loads correctly", "[yaml][ur3e]")
{
  const std::string path = source_path("config/models/ur3e_robot.yaml");
  auto params_opt = ParameterRobotModel::load_parameters_from_yaml(path);
  REQUIRE(params_opt.has_value());

  const RobotParameters &p = *params_opt;
  REQUIRE(p.dof == 6u);
  REQUIRE(p.mass.size() == p.dof);
  REQUIRE(p.com.rows() == static_cast<int>(p.dof));
  REQUIRE(p.com.cols() == 3);
  REQUIRE(p.joint_position_bounds.second.size() == static_cast<int>(p.dof));
}

TEST_CASE("load_parameters_from_yaml: UR10e loads correctly", "[yaml][ur10e]")
{
  const std::string path = source_path("config/models/ur10e_robot.yaml");
  auto params_opt = ParameterRobotModel::load_parameters_from_yaml(path);
  REQUIRE(params_opt.has_value());

  const RobotParameters &p = *params_opt;
  REQUIRE(p.dof == 6u);
  REQUIRE(p.mass.size() == p.dof);
  REQUIRE(p.com.rows() == static_cast<int>(p.dof));
  REQUIRE(p.com.cols() == 3);
  REQUIRE(p.joint_position_bounds.second.size() == static_cast<int>(p.dof));
}

TEST_CASE("load_parameters_from_yaml: breeding_blanket_handling_robot loads correctly", "[yaml][breeding]")
{
  const std::string path = source_path("config/models/breeding_blanket_handling_robot.yaml");
  auto params_opt = ParameterRobotModel::load_parameters_from_yaml(path);
  REQUIRE(params_opt.has_value());

  const RobotParameters &p = *params_opt;
  REQUIRE(p.dof == 7u);
  REQUIRE(p.mass.size() == p.dof);
  REQUIRE(p.com.rows() == static_cast<int>(p.dof));
  REQUIRE(p.com.cols() == 3);
  // joint_1 is prismatic
  REQUIRE(p.is_joint_revolute.size() == p.dof);
  REQUIRE(p.is_joint_revolute[0] == false);  // prismatic
}

TEST_CASE("load_parameters_from_yaml: non-existent file returns nullopt", "[yaml][error]")
{
  const std::string path = source_path("config/models/this_file_does_not_exist.yaml");
  auto params_opt = ParameterRobotModel::load_parameters_from_yaml(path);
  REQUIRE(!params_opt.has_value());
}
