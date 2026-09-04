#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sdu_controllers/controllers/operational_space_controller.hpp>
#include <sdu_controllers/math/forward_dynamics.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/math/trajectory_generation.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>
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
  auto robot_model = std::make_shared<models::URRobotModel>(models::URRobotModel::RobotType::ur5e);
  double freq = 500.0;
  double dt = 1.0 / freq;
  const double total_t = 4.0;   // seconds (two full circles)
  const size_t steps = static_cast<size_t>(total_t * freq);

  // Circle parameters
  const double radius = 0.05;               // 5 cm
  const double circle_freq = 0.5;            // Hz
  const double omega = 2.0 * pi * circle_freq;
  const double ramp_time = 0.3;              // s

  double Kp_pos_value = 16250.0;
  double Kp_orient_value = 16250.0;
  double Kd_pos_value = 200.0;
  double Kd_orient_value = 3.0;
  double N_value = 1;
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

  // ZYZ mode (default): x_d is [position(3), ZYZ angles(3)].
  controllers::OperationalSpaceController osc_controller(Kp, Kd, robot_model);

  VectorXd x_d(6);
  VectorXd dx_d(6);
  VectorXd ddx_d(6);

  VectorXd q(ROBOT_DOF);
  VectorXd dq(ROBOT_DOF);
  Vector<double, 6> he = VectorXd::Zero(6);
  q << 0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0;
  dq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Circle centre and fixed orientation derived from the initial end-effector pose.
  Matrix4d T0 = robot_model->get_fk_solver().forward_kinematics(q);
  Vector3d center = T0.block<3, 1>(0, 3);
  center[0] -= radius;   // shift so the trajectory starts at the robot's initial position
  Vector3d rot_zyz_d = T0.topLeftCorner<3, 3>().eulerAngles(2, 1, 2);

  // Control loop
  for (size_t step = 0; step < steps; ++step)
  {
    const double t = step * dt;

    // Desired Cartesian pose, velocity and acceleration
    Vector3d pos_d, dpos_d, ddpos_d;
    math::circular_trajectory(center, radius, omega, t, ramp_time, pos_d, dpos_d, ddpos_d);

    x_d << pos_d, rot_zyz_d;
    dx_d << dpos_d, Vector3d::Zero();
    ddx_d << ddpos_d, Vector3d::Zero();

    // Add noise to q and dq
    VectorXd q_meas = q;
    VectorXd dq_meas = dq;
    //add_noise_to_vector(q_meas, 0.0, 0.001);
    //add_noise_to_vector(dq_meas, 0.0, 0.001);

    // Controller
    osc_controller.step(x_d, dx_d, ddx_d, q_meas, dq_meas);
    VectorXd y = osc_controller.get_output();
    VectorXd tau = robot_model->inverse_dynamics(q_meas, dq_meas, y, he);

    // Simulation
    VectorXd ddq = robot_model->forward_dynamics(q, dq, tau);
    // integrate to get velocity
    dq += ddq * dt;
    // integrate to get position
    q += dq * dt;

    MatrixXd T = robot_model->get_fk_solver().forward_kinematics(q);
    VectorXd pos = T.block<3, 1>(0, 3);
    Matrix3d rot_mat = T.block<3,3>(0, 0);
    Vector3d rpy_zyz = rot_mat.eulerAngles(2, 1, 2); // ZYZ representation
    VectorXd temp(q.size()+pos.size()+rpy_zyz.size());
    temp << q, pos, rpy_zyz;
    csv_writer << eigen_to_std_vector(temp);
  }
  output_filestream.close();
  std::cout << "Simulation complete. Output written to output_cartesian.csv" << std::endl;
}
