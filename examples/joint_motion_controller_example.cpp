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

using namespace csv;
using namespace Eigen;
using namespace sdu_controllers;
using namespace sdu_controllers::utils;

int main()
{
  // Setup reading of input trajectory from csv and writing of output trajectory to csv.
  CSVReader reader("../../examples/data/trajectory.csv");
  std::ofstream output_filestream;
  output_filestream.open("output.csv");
  auto csv_writer = make_csv_writer(output_filestream);

  // Initialize robot model and parameters
  auto robot_model = std::make_shared<models::URRobotModel>(UR5e);
  double freq = 500.0;
  double dt = 1.0 / freq;
  double Kp_value = 100.0;
  double Kd_value = 2 * sqrt(Kp_value);
  double N_value = 1;
  uint16_t ROBOT_DOF = robot_model->get_dof();
  VectorXd Kp_vec = VectorXd::Ones(ROBOT_DOF) * Kp_value;
  VectorXd Kd_vec = VectorXd::Ones(ROBOT_DOF) * Kd_value;
  VectorXd N_vec = VectorXd::Ones(ROBOT_DOF) * N_value;

  controllers::PDController pd_controller(Kp_vec.asDiagonal(), Kd_vec.asDiagonal(), N_vec.asDiagonal());
  math::InverseDynamicsJointSpace inv_dyn_jnt_space(robot_model);
  math::ForwardDynamics fwd_dyn(robot_model, dt);

  VectorXd q_d(ROBOT_DOF);
  VectorXd dq_d(ROBOT_DOF);
  VectorXd ddq_d(ROBOT_DOF);

  VectorXd q(ROBOT_DOF);
  VectorXd dq(ROBOT_DOF);
  q << 0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0;
  dq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Control loop
  for (const auto& row : reader)
  {
    // Desired configuration
    assert(row.size() == q_d.size()+dq_d.size()+ddq_d.size());
    for (Index i = 0; i < q_d.size(); i++)
    {
      q_d[i] = row[i].get<double>();
      dq_d[i] = row[i+ROBOT_DOF].get<double>();
      ddq_d[i] = row[i+(2*ROBOT_DOF)].get<double>();
    }

    // Add noise to q and dq
    VectorXd q_meas = q;
    VectorXd dq_meas = dq;
    addNoiseToVector(q_meas, 0.0, 0.2);
    addNoiseToVector(dq_meas, 0.0, 0.2);

    // Controller
    pd_controller.step(q_d, dq_d, ddq_d, q_meas, dq_meas);
    VectorXd y = pd_controller.get_output();
    std::cout << "y: " << y << std::endl;
    VectorXd tau = inv_dyn_jnt_space.inverse_dynamics(y, q_meas, dq_meas);
    std::cout << "tau: " << tau << std::endl;

    // Simulation
    VectorXd ddq = fwd_dyn.forward_dynamics(q, dq, tau);
    // integrate to get velocity
    dq += ddq * dt;
    // integrate to get position
    q += dq * dt;

    std::cout << "q:" << q << std::endl;
    csv_writer << eigenToStdVector(q);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  output_filestream.close();
}
