#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sdu_controllers/controllers/impedance_controller.hpp>
#include <sdu_controllers/math/forward_dynamics.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>
#include <sdu_controllers/utils/utility.hpp>

using namespace csv;
using namespace Eigen;
using namespace sdu_controllers;
using namespace sdu_controllers::utils;

constexpr double pi = 3.14159265358979323846;

/**
 * Generate the desired Cartesian position on a circle in the XY plane.
 *
 * @param center     Circle centre (x0, y0, z0) in metres.
 * @param radius     Circle radius in metres.
 * @param omega      Angular velocity (rad/s).
 * @param t          Current time (s).
 * @param ramp_time  Ramp duration (s) for angular speed from 0 to omega.
 * @param pos        Output: desired position (3-vector).
 * @param vel        Output: desired velocity (3-vector).
 * @param acc        Output: desired acceleration (3-vector).
 */
static void circle_trajectory(
    const Vector3d &center,
    double radius,
    double omega,
    double t,
    double ramp_time,
    Vector3d &pos,
    Vector3d &vel,
    Vector3d &acc)
{
  // Quintic angular speed ramp: w(t) = omega * (10u^3 - 15u^4 + 6u^5), u=t/ramp_time.
  // This is C2 continuous (zero accel/jerk at ramp start/end), reducing torque overshoot.
  double theta = 0.0;
  double w = omega;
  double alpha = 0.0;

  if (ramp_time > 0.0 && t < ramp_time)
  {
    double u = t / ramp_time;
    if (u < 0.0)
    {
      u = 0.0;
    }
    else if (u > 1.0)
    {
      u = 1.0;
    }

    w = omega * (10.0 * u * u * u - 15.0 * u * u * u * u + 6.0 * u * u * u * u * u);
    alpha = omega * (30.0 * u * u - 60.0 * u * u * u + 30.0 * u * u * u * u) / ramp_time;
    theta = omega * ramp_time * (2.5 * u * u * u * u - 3.0 * u * u * u * u * u + u * u * u * u * u * u);
  }
  else
  {
    // Keep phase continuous with ramp profile at t=ramp_time.
    theta = omega * (t - 0.5 * ramp_time);
  }

  const double c = std::cos(theta);
  const double s = std::sin(theta);

  pos << center[0] + radius * c,
         center[1] + radius * s,
         center[2];

  vel << -radius * s * w,
          radius * c * w,
          0.0;

  acc << -radius * c * w * w - radius * s * alpha,
         -radius * s * w * w + radius * c * alpha,
          0.0;
}

int main()
{
  // -------------------------------------------------------------------------
  // Setup
  // -------------------------------------------------------------------------
  std::ofstream output_filestream;
  output_filestream.open("output_impedance.csv");
  auto csv_writer = make_csv_writer(output_filestream);
  std::vector<std::string> header = {
      "t", "q0", "q1", "q2", "q3", "q4", "q5",
      "x", "y", "z", "zyz0", "zyz1", "zyz2",
      "x_d", "y_d", "z_d"};
  csv_writer << header;

  auto robot_model = std::make_shared<models::URRobotModel>(models::URRobotModel::RobotType::ur5e);
  const uint16_t DOF = robot_model->get_dof();

  const double freq     = 500.0;
  const double dt       = 1.0 / freq;
  const double total_t  = 4.0;            // seconds (two full circles)
  const size_t steps    = static_cast<size_t>(total_t * freq);

  // Circle parameters
  const double radius       = 0.05;                    // 5 cm
  const double circle_freq  = 0.5;                     // Hz
  const double omega        = 2.0 * pi * circle_freq;
  const double ramp_time    = 0.3;                     // s

  // -------------------------------------------------------------------------
  // Controller gains
  // -------------------------------------------------------------------------
  const double Kp_pos    = 8000.0;
  const double Kp_orient = 15.0;

  MatrixXd Kp = MatrixXd::Zero(6, 6);
  Kp.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3) * Kp_pos;
  Kp.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3) * Kp_orient;

  // Desired inertia: 2.5 kg for translational axes, 0.5 kg*m2 for rotational
  VectorXd Md_pos = VectorXd::Ones(3) * 2.5;
  VectorXd Md_rot = VectorXd::Ones(3) * 0.5;
  MatrixXd Md = MatrixXd::Zero(6, 6);
  Md.block<3, 3>(0, 0) = Md_pos.asDiagonal();
  Md.block<3, 3>(3, 3) = Md_rot.asDiagonal();

  // Critical damping: D = 2 * sqrt(M * K)
  const double Kd_pos    = 2.0 * std::sqrt(Md_pos[0] * Kp_pos);
  const double Kd_orient = 2.0 * std::sqrt(Md_rot[0] * Kp_orient);

  MatrixXd Kd = MatrixXd::Zero(6, 6);
  Kd.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3) * Kd_pos;
  Kd.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3) * Kd_orient;

  controllers::ImpedanceController impedance_controller(
      Kp, Kd, Md, robot_model,
      controllers::ImpedanceController::OrientationRepresentation::QUATERNION);

  // -------------------------------------------------------------------------
  // Initial robot state
  // -------------------------------------------------------------------------
  VectorXd q(DOF), dq(DOF);
  q  << 0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0;   // standard ready position
  dq << 0.0,  0.0,     0.0,     0.0,     0.0,    0.0;

  // External forces/torques at the end-effector (zero – free-space motion)
  Vector<double, 6> he  = VectorXd::Zero(6);
  // Desired contact wrench (zero - purely motion tracking)
  VectorXd h_d_e = VectorXd::Zero(6);

  // -------------------------------------------------------------------------
  // Derive circle centre from initial end-effector pose
  // -------------------------------------------------------------------------
  Matrix4d T0     = robot_model->get_fk_solver().forward_kinematics(q);
  Vector3d center = T0.block<3, 1>(0, 3);
  center[0]      -= radius;   // shift so trajectory starts at robot's initial position

  // Desired orientation: constant at the initial FK orientation (as quaternion)
  Matrix3d rot0 = T0.topLeftCorner(3, 3);   
  Quaterniond quat_d(rot0);
  quat_d.normalize();
  // Pack as Vector4d [w, x, y, z] to match step() convention
  Vector4d quat_d_vec(quat_d.w(), quat_d.x(), quat_d.y(), quat_d.z());

  // -------------------------------------------------------------------------
  // Control loop
  // -------------------------------------------------------------------------
  for (size_t step = 0; step < steps; ++step)
  {
    const double t = step * dt;

    // Desired Cartesian pose, velocity and acceleration
    Vector3d pos_d, dpos_d, ddpos_d;
    circle_trajectory(center, radius, omega, t, ramp_time, pos_d, dpos_d, ddpos_d);

    VectorXd x_d(6),  dx_d(6),  ddx_d(6);
    x_d   << pos_d,    Vector3d::Zero();   // orientation part unused in QUATERNION mode
    dx_d  << dpos_d,   Vector3d::Zero();
    ddx_d << ddpos_d,  Vector3d::Zero();

    // Measured state (ideal sensor - no noise)
    VectorXd q_meas  = q;
    VectorXd dq_meas = dq;

    // Impedance controller step (QUATERNION mode)
    impedance_controller.step(x_d, dx_d, ddx_d, q_meas, dq_meas, h_d_e, quat_d_vec);
    VectorXd y = impedance_controller.get_output();   // joint accelerations

    // Convert joint accelerations to joint torques via inverse dynamics
    VectorXd tau = robot_model->inverse_dynamics(q_meas, dq_meas, y, he);

    // Simulate one time step with forward dynamics
    VectorXd ddq = robot_model->forward_dynamics(q, dq, tau);
    dq += ddq * dt;
    q  += dq  * dt;

    // Collect actual end-effector position for logging
    Matrix4d T_actual = robot_model->get_fk_solver().forward_kinematics(q);
    Vector3d pos_actual = T_actual.block<3, 1>(0, 3);
    Matrix3d rot_actual = T_actual.topLeftCorner(3, 3);
    Vector3d zyz_actual = rot_actual.eulerAngles(2, 1, 2);

    // Write: time | q (6) | x_actual (3) | zyz_actual (3) | x_d (3)
    VectorXd row(1 + DOF + 3 + 3 + 3);
    row << t, q, pos_actual, zyz_actual, pos_d;
    csv_writer << eigen_to_std_vector(row);
  }

  output_filestream.close();
  std::cout << "Simulation complete. Output written to output_impedance.csv" << std::endl;

  return 0;
}
