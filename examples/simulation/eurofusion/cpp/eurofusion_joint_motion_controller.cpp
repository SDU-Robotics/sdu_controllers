#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sdu_controllers/controllers/pid_controller.hpp>
#include <sdu_controllers/models/breeding_blanket_handling_robot_model.hpp>
#include <sdu_controllers/utils/utility.hpp>

using namespace csv;
using namespace Eigen;
using namespace sdu_controllers;
using namespace sdu_controllers::utils;

int main()
{
  // Setup writing of output trajectory to csv.
  std::ofstream output_filestream;
  output_filestream.open("output_eurofusion_joint.csv");
  auto csv_writer = make_csv_writer(output_filestream);

  // Initialize robot model and parameters
  auto robot_model = std::make_shared<models::BreedingBlanketHandlingRobotModel>();

  // Pick up
  double mass = 1000;
  Vector3d com = {1., 2., 3.,};
  Matrix3d inertia;
  inertia << 1., 2., 3.,
             2., 4., 5.,
             3., 5., 6.;
  inertia << inertia * 1e5;

  robot_model->set_tcp_mass(mass, com, inertia);

  //
  double freq = 1000.0;
  double dt = 1.0 / freq;

  double Kp_value = 1000.0;
  double Ki_value = 50.;
  // double Kd_value = 2.0 * sqrt(Kp_value);
  double Kd_value = 100.;
  double N_value = 1.0;
  uint16_t ROBOT_DOF = robot_model->get_dof();
  VectorXd Kp_vec = VectorXd::Ones(ROBOT_DOF) * Kp_value;
  VectorXd Ki_vec = VectorXd::Ones(ROBOT_DOF) * Ki_value;
  VectorXd Kd_vec = VectorXd::Ones(ROBOT_DOF) * Kd_value;
  VectorXd N_vec = VectorXd::Ones(ROBOT_DOF) * N_value;

  VectorXd u_max(ROBOT_DOF);
  // Breeding blanket robot max and min torque
  u_max << 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0;
  VectorXd u_min = -u_max;

  controllers::PIDController pid_controller(Kp_vec.asDiagonal(), Ki_vec.asDiagonal(),
    Kd_vec.asDiagonal(), N_vec.asDiagonal(), dt, u_min, u_max);

  //math::RecursiveNewtonEuler rnea(robot_model);
  //Vector3d z0;
  //z0 << 0.0, 0.0, -1.0;
  //rnea.set_z0(z0);
  //math::InverseDynamicsJointSpace inv_dyn(robot_model);
  //math::ForwardDynamics fwd_dyn(robot_model);

  VectorXd q_d(ROBOT_DOF);
  VectorXd dq_d(ROBOT_DOF);
  VectorXd ddq_d(ROBOT_DOF);

  VectorXd q(ROBOT_DOF);
  VectorXd dq(ROBOT_DOF);
  VectorXd ddq(ROBOT_DOF);

  q << 0.0027, 0.0004, 0.0, 0.0, -0.0004, -0.062, 0.0;
  dq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ddq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  csv_writer << eigen_to_std_vector(q);

  std::cout << q << std::endl;

  VectorXd tau(ROBOT_DOF);
  Vector<double, 6> he = VectorXd::Zero(6);

  // Read input trajectory from file
  std::vector<std::vector<double>> input_trajectory = get_trajectory_from_file(utils::data_path("joint_trajectory_safe_bb.csv"));

  // Control loop
  for (const std::vector<double>& trajectory_point : input_trajectory)
  {
    // Desired
    for (Index i = 0; i < q_d.size(); i++)
    {
      q_d[i] = trajectory_point[1+i];
      dq_d[i] = trajectory_point[1+i+ROBOT_DOF];
      ddq_d[i] = trajectory_point[1+i+(2*ROBOT_DOF)];
    }
    std::cout << "q_d: " << q_d << std::endl;

    // Add noise to q and dq
    VectorXd q_meas = q;
    VectorXd dq_meas = dq;
    //add_noise_to_vector(q_meas, 0.0, 0.002);
    //add_noise_to_vector(dq_meas, 0.0, 0.002);

    // Controller
    VectorXd u_ff = ddq_d; // acceleration as feedforward.
    // VectorXd u_ff = robot_model->get_gravity(q_meas); // feedforward with gravity compensation.
    pid_controller.step(q_d, dq_d, u_ff, q_meas, dq_meas);
    VectorXd y = pid_controller.get_output();
    tau = robot_model->inverse_dynamics(q_meas, dq_meas, y, he);

    // Simulation
    // ddq = rnea.forward_dynamics(q, dq, tau, he);
    ddq = robot_model->forward_dynamics(q, dq, tau);
    // integrate to get velocity
    dq += ddq * dt;
    // integrate to get position
    q += dq * dt;

    std::cout << "q: " << q << std::endl;
    csv_writer << eigen_to_std_vector(q);
  }
  output_filestream.close();
}
