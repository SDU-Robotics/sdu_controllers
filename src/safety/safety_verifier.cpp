#include <iostream>
#include <sdu_controllers/safety/safety_verifier.hpp>
#include <utility>
#include <stdexcept>

using namespace Eigen;

namespace sdu_controllers::safety
{
  SafetyVerifier::SafetyVerifier(std::shared_ptr<models::RobotModel> robot_model) : robot_model_(std::move(robot_model))
  {
  }

  bool SafetyVerifier::verify_trajectory_safety(const std::vector<std::vector<double>> &trajectory)
  {
    VectorXd q_d(robot_model_->get_dof());
    VectorXd dq_d(robot_model_->get_dof());
    VectorXd ddq_d(robot_model_->get_dof());

    if (!trajectory.empty())
    {
      for (const auto &trajectory_point : trajectory)
      {
        // Desired
        for (Index i = 0; i < q_d.size(); i++)
        {
          q_d[i] = trajectory_point[i];
          dq_d[i] = trajectory_point[i+robot_model_->get_dof()];
          ddq_d[i] = trajectory_point[i+(2*robot_model_->get_dof())];
        }
        if (!check_joint_pos_limits(q_d))
        {
          std::cerr << "joint position bounds violated!" << std::endl;
          return false;
        }
        if (!check_joint_vel_limits(dq_d))
        {
          std::cerr << "joint velocity bounds violated!" << std::endl;
          return false;
        }
        if (!check_joint_acc_limits(ddq_d))
        {
          std::cerr << "joint acceleration bounds violated!" << std::endl;
          return false;
        }
      }
      return true;
    }
    else
    {
      std::cerr << "input trajectory is empty!" << std::endl;
      return false;
    }
  }

  bool check_limits(const VectorXd& vec, const VectorXd& low_limit, const VectorXd& high_limit)
  {
    bool is_within_bounds = false;
    for (Index i = 0; i < vec.size(); i++)
      is_within_bounds = utils::is_within_bounds(vec[i], low_limit[i], high_limit[i]);
    return is_within_bounds;
  }

  bool SafetyVerifier::check_joint_pos_limits(const VectorXd& q) const
  {
    const std::pair<VectorXd, VectorXd> q_bounds = robot_model_->get_joint_pos_bounds();
    return check_limits(q, q_bounds.first, q_bounds.second);
  }

  bool SafetyVerifier::check_joint_vel_limits(const VectorXd& dq) const
  {
    const std::pair<VectorXd, VectorXd> dq_bounds = robot_model_->get_joint_vel_bounds();
    return check_limits(dq, dq_bounds.first, dq_bounds.second);
  }

  bool SafetyVerifier::check_joint_acc_limits(const VectorXd& ddq) const
  {
    const std::pair<VectorXd, VectorXd> ddq_bounds = robot_model_->get_joint_acc_bounds();
    return check_limits(ddq, ddq_bounds.first, ddq_bounds.second);
  }

  bool SafetyVerifier::check_joint_torque_limits(const VectorXd& tau) const
  {
    const std::pair<VectorXd, VectorXd> torque_bounds = robot_model_->get_joint_torque_bounds();
    return check_limits(tau, torque_bounds.first, torque_bounds.second);
  }

}  // namespace sdu_controllers::safety
