#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sdu_controllers/controllers/pid_controller.hpp>
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
  output_filestream.open("output.csv");
  auto csv_writer = make_csv_writer(output_filestream);

  // Initialize robot model and parameters
  auto robot_model = std::make_shared<models::URRobotModel>(models::URRobotModel::RobotType::ur5e);
  double freq = 500.0;
  double dt = 1.0 / freq;
  const double total_t = 10.0;  // seconds
  const size_t steps = static_cast<size_t>(total_t * freq);

  // Sinusoidal joint trajectory: q_d(t) = q0 + amplitude * sin(omega * t), same on every joint.
  const double amplitude = 0.1;  // rad
  const double traj_freq = 0.2;  // Hz
  const double omega = 2.0 * pi * traj_freq;

  double Kp_value = 1000.0;
  double Ki_value = 100.0;
  double Kd_value = 2 * sqrt(Kp_value);
  double N_value = 1;
  uint16_t ROBOT_DOF = robot_model->get_dof();
  VectorXd Kp_vec = VectorXd::Ones(ROBOT_DOF) * Kp_value;
  VectorXd Ki_vec = VectorXd::Ones(ROBOT_DOF) * Ki_value;
  VectorXd Kd_vec = VectorXd::Ones(ROBOT_DOF) * Kd_value;
  VectorXd N_vec = VectorXd::Ones(ROBOT_DOF) * N_value;

  VectorXd u_max(ROBOT_DOF);
  // UR5e max and min torque see https://www.universal-robots.com/articles/ur/robot-care-maintenance/max-joint-torques-cb3-and-e-series/
  u_max << 150.0, 150.0, 150.0, 28.0, 28.0, 28.0;
  VectorXd u_min = -u_max;

  controllers::PIDController pid_controller(Kp_vec.asDiagonal(), Ki_vec.asDiagonal(),
    Kd_vec.asDiagonal(), N_vec.asDiagonal(), dt, u_min, u_max);

  VectorXd q0(ROBOT_DOF);
  q0 << 0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0;

  VectorXd q = q0;
  VectorXd dq = VectorXd::Zero(ROBOT_DOF);
  Vector<double, 6> he = VectorXd::Zero(6);

  safety::SafetyVerifier safety_verifier(robot_model);

  // Control loop
  for (size_t step = 0; step < steps; step++)
  {
    double t = step * dt;

    // Desired
    VectorXd q_d = (q0.array() + amplitude * std::sin(omega * t)).matrix();
    VectorXd dq_d = VectorXd::Ones(ROBOT_DOF) * (amplitude * omega * std::cos(omega * t));
    VectorXd ddq_d = VectorXd::Ones(ROBOT_DOF) * (-amplitude * omega * omega * std::sin(omega * t));

    // Online safety verification of the desired sample.
    //  - checks joint position, velocity and acceleration limits.
    if (!safety_verifier.check_joint_pos_limits(q_d) || !safety_verifier.check_joint_vel_limits(dq_d) ||
        !safety_verifier.check_joint_acc_limits(ddq_d))
    {
      std::cerr << "desired joint trajectory is not safe!" << std::endl;
      break;
    }

    // Controller
    VectorXd u_ff = ddq_d; // acceleration as feedforward.
    // VectorXd u_ff = robot_model->get_gravity(q); // feedforward with gravity compensation.
    pid_controller.step(q_d, dq_d, u_ff, q, dq);
    VectorXd y = pid_controller.get_output();
    VectorXd tau = robot_model->inverse_dynamics(q, dq, y, he);

    // Simulation of the resulting motion, used here to validate the tracking performance.
    VectorXd ddq = robot_model->forward_dynamics(q, dq, tau);
    // integrate to get velocity
    dq += ddq * dt;
    // integrate to get position
    q += dq * dt;

    VectorXd row(1 + 3 * ROBOT_DOF);
    row << t, q, q_d, tau;
    csv_writer << eigen_to_std_vector(row);
  }
  output_filestream.close();

  std::cout << "Simulation complete." << std::endl;
  return 0;
}
