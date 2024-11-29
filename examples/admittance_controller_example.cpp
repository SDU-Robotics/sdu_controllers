#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sdu_controllers/controllers/admittance_controller_position.hpp>
#include <sdu_controllers/utils/utility.hpp>

using namespace std;
using namespace csv;
using namespace Eigen;
using namespace sdu_controllers;
using namespace sdu_controllers::utils;


Vector3d get_circle_target(const Vector3d &pose, double timestep, double radius = 0.1, double freq = 0.5)
{
  Vector3d circle_target;
  circle_target[0] = pose[0] + radius * cos(2 * M_PI * freq * timestep);
  circle_target[1] = pose[1] + radius * sin(2 * M_PI * freq * timestep);
  circle_target[2] = pose[2];
  return circle_target;
}

int main()
{
  double frequency = 500;
  double dt = 1. / frequency;

  auto t_steps = static_cast<uint16_t>(2 * frequency);

  // Create file outputs
  std::ofstream output_filestream_ref;
  string output_filename_ref = "output_admittance_reference.csv";
  output_filestream_ref.open(output_filename_ref);
  auto csv_writer_ref = make_csv_writer(output_filestream_ref);


  std::ofstream output_filestream_adm;
  string output_filename_adm = "output_admittance.csv";
  output_filestream_adm.open(output_filename_adm);
  auto csv_writer_adm = make_csv_writer(output_filestream_adm);

  cout << "Writing to " << output_filename_adm << endl;

  Vector3d start_position(0.3, 0.3, 0.3);
  VectorXd x_desired;

  // Create reference trajectory (desired trajectory)
  vector<Vector3d> ref_traj;
  for (uint16_t t = 0; t < t_steps; t++)
  {
    x_desired = get_circle_target(start_position, t * dt);
    csv_writer_ref << eigen_to_std_vector(x_desired);
  }

  // Compute admittance trajectory
  VectorXd u;
  controllers::AdmittanceControllerPosition adm_controller;
  adm_controller.set_mass_matrix_position(Vector3d(22.5, 22.5, 22.5));
  adm_controller.set_stiffness_matrix_position(Vector3d(54, 54, 54));
  adm_controller.set_damping_matrix_position(Vector3d(160, 160, 160));

  adm_controller.set_mass_matrix_orientation(Vector3d(0.25, 0.25, 0.25));
  adm_controller.set_stiffness_matrix_orientation(Vector3d(10, 10, 10));
  adm_controller.set_damping_matrix_orientation(Vector3d(10, 10, 10));

  Vector3d adm_pos = get_circle_target(start_position, 0);
  Quaterniond quat_init(1., 0., 0., 0.);
  Vector3d f = Vector3d::Zero();
  Vector3d mu = Vector3d::Zero();

  vector<Vector3d> adm_traj;
  for (uint16_t t = 0; t < t_steps; t++)
  {
    x_desired = get_circle_target(start_position, t * dt);
    if (adm_pos[0] > 0.29 && adm_pos[0] < 0.31 && adm_pos[1] > 0.39 && adm_pos[1] < 0.41)
    {
      f[1] = 40;
    }
    else
    {
      f[1] = 0;
    }

    // Step controller
    adm_controller.step(f, mu, x_desired, quat_init);
    u = adm_controller.get_output();

    adm_pos[0] = u[0];
    adm_pos[1] = u[1];
    adm_pos[2] = u[2];

    csv_writer_adm << eigen_to_std_vector(u);
  }

  output_filestream_ref.close();
  output_filestream_adm.close();
  return 0;
}