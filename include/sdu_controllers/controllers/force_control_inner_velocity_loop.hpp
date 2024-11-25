#pragma once
#ifndef FORCE_CONTROL_INNER_VELOCITY_LOOP_HPP
#define FORCE_CONTROL_INNER_VELOCITY_LOOP_HPP

#include <memory>
#include <Eigen/Dense>
#include <sdu_controllers/controllers/controller.hpp>
#include <sdu_controllers/models/robot_model.hpp>

namespace sdu_controllers::controllers
{
  class ForceControlInnerVelocityLoop : public Controller
  {
  public:
    explicit ForceControlInnerVelocityLoop(
       Eigen::MatrixXd Kp,
       Eigen::MatrixXd Kd,
       Eigen::MatrixXd Md,
       Eigen::MatrixXd Kf,
       std::shared_ptr<models::RobotModel> robot_model);

   void step(
      const Eigen::VectorXd &f_d,
      const Eigen::VectorXd &f_e,
      const Eigen::VectorXd &q,
      const Eigen::VectorXd &dq);

    Eigen::VectorXd get_output() override;

    void reset() override;

  private:
    Eigen::VectorXd y_;
    Eigen::MatrixXd Kp_, Kd_, Md_, Mdinv_, Kf_;
    std::shared_ptr<models::RobotModel> robot_model_;
  };
}


#endif //FORCE_CONTROL_INNER_VELOCITY_LOOP_HPP
