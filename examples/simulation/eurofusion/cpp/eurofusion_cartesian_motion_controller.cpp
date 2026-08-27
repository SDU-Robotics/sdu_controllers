#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sdu_controllers/controllers/operational_space_controller.hpp>
#include <sdu_controllers/math/forward_dynamics.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/models/parameter_robot_model.hpp>
#include <sdu_controllers/safety/safety_verifier.hpp>
#include <sdu_controllers/utils/utility.hpp>

using namespace csv;
using namespace Eigen;
using namespace sdu_controllers;
using namespace sdu_controllers::utils;

constexpr double pi = 3.14159265358979323846;

int main()
{
  // Setup writing of output trajectory to csv.
  std::ofstream output_filestream;
  output_filestream.open("output_cartesian.csv");
  auto csv_writer = make_csv_writer(output_filestream);

  // Initialize robot model and parameters
  auto robot_model = std::make_shared<models::ParameterRobotModel>(utils::ConfigFolder::find_config_file("breeding_blanket_handling_robot_fixed.yaml"));

  double freq = 1000.0;
  double dt = 1.0 / freq;

  double Kp_pos_value = 10.0;
  double Kp_orient_value = 1;
  double Kd_pos_value = 20.0;
  double Kd_orient_value = 2;
  double N_value = 0;
  uint16_t ROBOT_DOF = robot_model->get_dof();

  VectorXd Kp_pos_vec = VectorXd::Ones(3) * Kp_pos_value;
  VectorXd Kp_orient_vec = VectorXd::Ones(3) * Kp_orient_value;
  VectorXd Kd_pos_vec = VectorXd::Ones(3) * Kd_pos_value;
  VectorXd Kd_orient_vec = VectorXd::Ones(3) * Kd_orient_value;
  VectorXd N_vec = VectorXd::Ones(6) * N_value;

  MatrixXd Kp = MatrixXd::Zero(6, 6);
  Kp.setIdentity();
  Kp.block<3, 3>(0,0) = Kp_pos_vec.asDiagonal();
  Kp.block<3, 3>(3,3) = Kp_orient_vec.asDiagonal();
  MatrixXd Kd = MatrixXd::Zero(6, 6);
  Kd.setIdentity();
  Kd.block<3, 3>(0,0) = Kd_pos_vec.asDiagonal();
  Kd.block<3, 3>(3,3) = Kd_orient_vec.asDiagonal();

  controllers::OperationalSpaceController osc_controller(Kp, Kd, robot_model);
  osc_controller.set_kappa(1e-2);
  //math::InverseDynamicsJointSpace inv_dyn_jnt_space(robot_model);
  //math::ForwardDynamics fwd_dyn(robot_model);

  VectorXd x_d(6);
  VectorXd dx_d(6);
  VectorXd ddx_d(6);

  VectorXd q(ROBOT_DOF), q0(ROBOT_DOF);
  VectorXd dq(ROBOT_DOF);
  Vector<double, 6> he = VectorXd::Zero(6);
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  q0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  dq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // output to csv
  output_filestream << "q0,q1,q2,q3,q4,q5,x,y,z,rx,ry,rz" << std::endl;;

  // Stay put at the initial position
  // Control loop for 10s
  for (size_t j=0; j < freq * 20.; j++)  // (const std::vector<double>& trajectory_point : input_trajectory)
  {
    if (j % int(freq) == 0)
      std::cout << "Time: " << j * dt << std::endl;

    // Desired
    Eigen::Matrix4d T0 = robot_model->get_fk_solver().forward_kinematics(q0);
    VectorXd pos0 = T0.block<3, 1>(0, 3);
    Matrix3d rot_mat0 = T0.block<3,3>(0, 0);
    Vector3d rpy_zyz0 = rot_mat0.eulerAngles(2, 1, 2); // ZYZ representation

    Vector3d pos_desired = pos0;
    pos_desired[0] += 0.1;
    pos_desired[1] += -0.1;
    pos_desired[2] += 2;

    Vector3d rpy_zyz_desired = rpy_zyz0;
    rpy_zyz_desired[0] += pi / 8;
    rpy_zyz_desired[1] += 0;
    rpy_zyz_desired[2] += pi / 2;

    x_d << pos_desired, rpy_zyz_desired;
    dx_d.setZero();
    ddx_d.setZero();

    // std::cout << "x_d: " << x_d << std::endl;

    // Add noise to q and dq
    VectorXd q_meas = q;
    VectorXd dq_meas = dq;
    //add_noise_to_vector(q_meas, 0.0, 0.001);
    //add_noise_to_vector(dq_meas, 0.0, 0.001);

    // std::cout << "before all fk" << std::endl;
    // std::vector<Eigen::Matrix4d> TTT = robot_model->get_fk_solver().forward_kinematics_all(q0);
    // std::cout << "after all fk" << std::endl;
    // std::cout << TTT.at(4) << std::endl;

    // Controller
    osc_controller.step(x_d, dx_d, ddx_d, q_meas, dq_meas);
    VectorXd y = osc_controller.get_output();
    // std::cout << "y: " << y << std::endl;
    VectorXd tau = robot_model->inverse_dynamics(q_meas, dq_meas, y, he);
    // std::cout << "tau: " << tau << std::endl;

    // Simulation
    VectorXd ddq = robot_model->forward_dynamics(q, dq, tau);
    // integrate to get velocity
    dq += ddq * dt;
    // integrate to get position
    q += dq * dt;

    // std::cout << "q:" << q << std::endl;
    MatrixXd T = robot_model->get_fk_solver().forward_kinematics(q);
    VectorXd pos = T.block<3, 1>(0, 3);
    // std::cout << "pos:" << pos << std::endl;
    Matrix3d rot_mat = T.block<3,3>(0, 0);
    Vector3d rpy_zyz = rot_mat.eulerAngles(2, 1, 2); // ZYZ representation
    // std::cout << "rpy_zyz:" << rpy_zyz << std::endl;
    VectorXd temp(q.size()+pos.size()+rpy_zyz.size());
    temp << q, pos, rpy_zyz;
    csv_writer << eigen_to_std_vector(temp);
  }
  output_filestream.close();
}
