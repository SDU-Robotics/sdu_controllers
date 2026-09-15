#pragma once
#ifndef SDU_CONTROLLERS_HAL_EUROSUFION_HPP
#define SDU_CONTROLLERS_HAL_EUROSUFION_HPP

#include <Eigen/Dense>
#include <sdu_controllers/hal/robot.hpp>
#include <sdu_controllers/math/pose.hpp>
#include <string>
#include <vector>
#include <thread>


namespace sdu_controllers::hal
{

  class EurofusionRobot : Robot
  {

   public:
    EurofusionRobot();
    Eigen::VectorXd generalized_tau_2_motor_tau(const Eigen::VectorXd& tau_g, const Eigen::VectorXd& q, const Eigen::VectorXd& qd);


  private:
    double joint1_gearing(double tau_1);
    double joint2_gearing(double tau_2, double q_2);
    double joint3_gearing(double tau_3);
    double joint4_gearing(double tau_4, double q_4);
    double joint5_gearing(double tau_5);
    double joint6_gearing(double tau_6);
    Eigen::Vector2d joint7_gearing(double tau_7, double q_7, double qd_7);

  };





  


}  // namespace sdu_controllers::hal

#endif  // SDU_CONTROLLERS_HAL_EUROSUFION_HPP
