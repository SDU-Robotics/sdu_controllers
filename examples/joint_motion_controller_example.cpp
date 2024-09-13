#include <Eigen/Dense>
#include <iostream>
#include <sdu_controllers/controllers/pd_controller.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/models/breeding_blanket_handling_robot_model.hpp>
#include <sdu_controllers/models/robot_model.hpp>

using namespace Eigen;
using namespace sdu_controllers;

int main()
{
  VectorXd q(7);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  VectorXd dq(7);
  dq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  VectorXd q_d(7);
  q_d << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  VectorXd dq_d(7);
  dq_d << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
  VectorXd ddq_d(7);
  ddq_d << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;

  std::shared_ptr<models::BreedingBlanketHandlingRobotModel> robot_model =
      std::make_shared<models::BreedingBlanketHandlingRobotModel>();
  double Kp_value = 0.1;
  double Kd_value = sqrt(Kp_value);
  VectorXd Kp = VectorXd::Ones(robot_model->get_dof()) * Kp_value;
  VectorXd Kd = VectorXd::Ones(robot_model->get_dof()) * Kd_value;

  controllers::PDController pd_controller(Kp, Kd);
  math::InverseDynamicsJointSpace inv_dyn_jnt_space(robot_model);
  pd_controller.step(q_d, dq_d, ddq_d, q, q_d);
  VectorXd y = pd_controller.get_output();
  std::cout << "y: " << y << std::endl;
  VectorXd tau = inv_dyn_jnt_space.inverse_dynamics(y, q, dq);
  std::cout << "tau: " << tau << std::endl;
}
