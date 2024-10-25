#pragma once
#ifndef SDU_CONTROLLERS_SAFETY_INTEGRITY_VERIFIER_HPP
#define SDU_CONTROLLERS_SAFETY_INTEGRITY_VERIFIER_HPP

#include <memory>
#include <sdu_controllers/models/robot_model.hpp>
#include <sdu_controllers/utils/utility.hpp>

namespace sdu_controllers::safety
{
  /**
   * This class implements methods for verifying the safety of the robot control.
   */
  class SafetyVerifier
  {
   public:
    explicit SafetyVerifier(std::shared_ptr<models::RobotModel> robot_model);
    ~SafetyVerifier() = default;

    bool verify_trajectory_safety(const std::vector<std::vector<double>> &trajectory);

    bool check_joint_pos_limits(const Eigen::VectorXd& q) const;

    bool check_joint_vel_limits(const Eigen::VectorXd& dq) const;

    bool check_joint_acc_limits(const Eigen::VectorXd& ddq) const;

    bool check_joint_torque_limits(const Eigen::VectorXd& tau) const;

   private:
    std::shared_ptr<models::RobotModel> robot_model_;
  };

}  // namespace sdu_controllers::safety

#endif  // SDU_CONTROLLERS_SAFETY_INTEGRITY_VERIFIER_HPP