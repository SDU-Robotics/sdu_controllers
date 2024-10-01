#include <sdu_controllers/controllers/pd_controller.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/math/forward_dynamics.hpp>
#include <sdu_controllers/models/ur_robot.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>
#include <sdu_controllers/utils/csv.hpp>
#include <sdu_controllers/utils/utility.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <random>

using namespace csv;
using namespace Eigen;
using namespace sdu_controllers;

int main()
{
  double freq = 500.0;
  double dt = 1.0 / freq;
  std::vector<std::vector<std::string>> trajectory = utils::read_csv("../../examples/data/trajectory.csv");

  auto robot_model = std::make_shared<models::URRobotModel>(UR5e);
  double Kp_value = 100.0;
  double Kd_value = 2*sqrt(Kp_value);
  double N_value = 1;
  VectorXd Kp_vec = VectorXd::Ones(robot_model->get_dof()) * Kp_value;
  VectorXd Kd_vec = VectorXd::Ones(robot_model->get_dof()) * Kd_value;
  VectorXd N_vec = VectorXd::Ones(robot_model->get_dof()) * N_value;

  controllers::PDController pd_controller(Kp_vec.asDiagonal(), Kd_vec.asDiagonal(), N_vec.asDiagonal());
  math::InverseDynamicsJointSpace inv_dyn_jnt_space(robot_model);
  math::ForwardDynamics fwd_dyn(robot_model, dt);

  // Define random generator with Gaussian distribution
  const double mean = 0.0;
  const double stddev = 0.2;
  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);

  std::ofstream output_filestream; // Can also use ofstream, etc.
  output_filestream.open("output.csv");
  auto csv_writer = make_csv_writer(output_filestream);

  VectorXd q_d(6);
  VectorXd dq_d(6);
  VectorXd ddq_d(6);

  VectorXd q(6);
  VectorXd dq(6);
  q << 0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0;
  dq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Control loop
  for (const auto& row : trajectory)
  {
    // Desired configuration
    assert(row.size() == q_d.size()+dq_d.size()+ddq_d.size());
    for (Index i = 0; i < q_d.size(); i++)
    {
      q_d[i] = stod(row[i]);
      dq_d[i] = stod(row[i+6]);
      ddq_d[i] = stod(row[i+12]);
    }

    // Add noise to q and dq
    VectorXd q_mes = q;
    for (Index i = 0; i < q_mes.size(); i++)
      q_mes[i] += dist(generator);
    VectorXd dq_mes = dq;
    for (Index i = 0; i < dq_mes.size(); i++)
      dq_mes[i] += dist(generator);

    // Controller
    pd_controller.step(q_d, dq_d, ddq_d, q_mes, dq_mes);
    VectorXd y = pd_controller.get_output();
    std::cout << "y: " << y << std::endl;
    VectorXd tau = inv_dyn_jnt_space.inverse_dynamics(y, q_mes, dq_mes);
    std::cout << "tau: " << tau << std::endl;

    // Simulation
    VectorXd ddq = fwd_dyn.forward_dynamics(q, dq, tau);
    // integrate to get velocity
    dq += ddq * dt;
    // integrate to get position
    q += dq * dt;

    std::cout << "q:" << q << std::endl;
    std::vector<double> q_std(q.data(), q.data() + q.size());
    csv_writer << q_std;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  output_filestream.close();
}
