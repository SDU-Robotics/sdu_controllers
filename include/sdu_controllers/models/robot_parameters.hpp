#pragma once

#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <utility>
#include <optional>
#include <string>

namespace sdu_controllers::models {

struct RobotParameters
{
  // degrees of freedom
  uint16_t dof;
  // forward kinematics model (either based on dh parameters or frames)
  std::shared_ptr<kinematics::ForwardKinematics> fk_model;

  // inertial
  std::vector<double> mass;
  Eigen::Matrix<double, Eigen::Dynamic, 3> com;
  std::vector<Eigen::Matrix3d> link_inertia;
  std::vector<bool> is_joint_revolute;
  Eigen::Vector3d g0{0, 0, -9.81};

  // joint limits
  std::pair<Eigen::VectorXd, Eigen::VectorXd> joint_position_bounds;
  Eigen::VectorXd joint_max_velocity;
  Eigen::VectorXd joint_max_acceleration;
  Eigen::VectorXd joint_max_torque;
};

} // namespace sdu_controllers::models
