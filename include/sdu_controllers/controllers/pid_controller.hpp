#pragma once
#ifndef SDU_CONTROLLERS_PID_CONTROLLER_HPP
#define SDU_CONTROLLERS_PID_CONTROLLER_HPP

#include <Eigen/Dense>
#include <sdu_controllers/controllers/controller.hpp>

namespace sdu_controllers::controllers
{
  class PIDController : public Controller
  {
   public:
    explicit PIDController(Eigen::MatrixXd Kp,  Eigen::MatrixXd Ki,
                           Eigen::MatrixXd Kd, Eigen::MatrixXd N, double dt, Eigen::VectorXd u_min, Eigen::VectorXd u_max);

    /**
     * @brief Step the execution of the controller.
     */
    void step(const Eigen::VectorXd& q_d, const Eigen::VectorXd& dq_d, const Eigen::VectorXd& u_ff, const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

    /**
     * @brief Get the output of the controller. Updates when the step() function is called.
     */
    Eigen::VectorXd get_output() override;

    /**
     * @brief Reset internal controller variables.
     */
    void reset() override;

   private:
    Eigen::VectorXd u_, integral_term_;
    Eigen::MatrixXd Kp_, Ki_, Kd_, N_;
    double dt_;

    // Minimum and maximum output limits for anti-windup
    Eigen::VectorXd u_min_;
    Eigen::VectorXd u_max_;
  };
}  // namespace sdu_controllers::controllers

#endif  // SDU_CONTROLLERS_PID_CONTROLLER_HPP
