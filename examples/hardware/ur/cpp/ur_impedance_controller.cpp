#include <Eigen/Dense>
#include <signal.h>
#include <fstream>
#include <limits>
#include <iostream>
#include <sdu_controllers/controllers/impedance_controller.hpp>
#include <sdu_controllers/math/rnea.hpp>
#include <sdu_controllers/hal/ur_robot.hpp>
#include <sdu_controllers/models/ur_robot_model.hpp>
#include <sdu_controllers/safety/safety_verifier.hpp>
#include <sdu_controllers/utils/utility.hpp>

using namespace csv;
using namespace std::chrono;
using namespace Eigen;
using namespace sdu_controllers;
using namespace sdu_controllers::utils;
using namespace sdu_controllers::math;
using namespace sdu_controllers::hal;

constexpr double pi = 3.14159265358979323846;

// Interrupt flag
bool flag_loop = true;
void raise_flag(int)
{
  flag_loop = false;
}

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

Eigen::VectorXd clamp_array(const Eigen::VectorXd& value, const Eigen::VectorXd& clamp_value)
{
  Eigen::VectorXd ret = value;
  Eigen::VectorXd pdiff = value - clamp_value;
  Eigen::VectorXd mdiff = value + clamp_value;

  for (int j = 0; j < value.size(); ++j) {
      if (pdiff[j] > 0) {
          ret[j] = clamp_value[j];
      } else if (mdiff[j] < 0) {
          ret[j] = -clamp_value[j];
      }
  }
  return ret;
}

/**
 * Apply a symmetric deadband to a wrench: any component with magnitude below
 * the corresponding threshold is set to zero. Components above the threshold are
 * passed through unchanged (not offset-shifted), so contact forces are preserved.
 *
 * The first three components are treated as forces (N) and the last three as
 * torques (Nm), each with its own threshold.
 *
 * @param wrench           Input wrench (forces in [0:3], torques in [3:6]).
 * @param force_threshold  Deadband half-width for the force components (N).
 * @param torque_threshold Deadband half-width for the torque components (Nm).
 * @returns the deadbanded wrench.
 */
Eigen::VectorXd apply_deadband(const Eigen::VectorXd& wrench, double force_threshold, double torque_threshold)
{
  Eigen::VectorXd ret = wrench;
  for (int j = 0; j < wrench.size(); ++j)
  {
    const double threshold = (j < 3) ? force_threshold : torque_threshold;
    if (std::abs(wrench[j]) < threshold)
    {
      ret[j] = 0.0;
    }
  }
  return ret;
}


int main(int argc, char* argv[])
{
  // Setup writing of output trajectory to csv.
  std::ofstream output_filestream;
  output_filestream.open("output.csv");
  auto csv_writer = make_csv_writer(output_filestream);
  // row << t, tau+tau_g, q, dq, currents, he, pos, rotvec, pos_d, y, tau_clamped
  std::vector<std::string> header = {"t", "tau0", "tau1", "tau2", "tau3", "tau4", "tau5", "q0", "q1", "q2", "q3", "q4", "q5", "dq0", "dq1", "dq2", "dq3", "dq4", "dq5", "current0", "current1", "current2", "current3", "current4", "current5", "he0", "he1", "he2", "he3", "he4", "he5", "x", "y", "z", "rot_x", "rot_y", "rot_z", "x_d", "y_d", "z_d", "y0", "y1", "y2", "y3", "y4", "y5", "tau_cmd0", "tau_cmd1", "tau_cmd2", "tau_cmd3", "tau_cmd4", "tau_cmd5"};
  csv_writer << header;

  std::string robot_ip = "127.0.0.1";
  if (argc > 1)
  {
    robot_ip = argv[1];
  }

  // Initialize robot model and parameters
  auto robot_model = std::make_shared<models::URRobotModel>(models::URRobotModel::RobotType::ur5e);
  const uint16_t DOF = robot_model->get_dof();

  const double freq     = 500.0;
  const double dt       = 1.0 / freq;
  const double total_t  = 100.0;            // seconds
  const size_t steps    = static_cast<size_t>(total_t * freq);

  // Circle parameters
  const double radius       = 0.05;                    // 5 cm
  const double circle_freq  = 0.5;                     // Hz
  const double omega        = 2.0 * M_PI * circle_freq;
  const double ramp_time    = 0.3;                     // s

  // Controller gains
  const double Kp_pos    = 1000.0; // 1000.0;
  const double Kp_orient = 50.0; // 10.0;

  MatrixXd Kp = MatrixXd::Zero(6, 6);
  Kp.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3) * Kp_pos;
  Kp.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3) * Kp_orient;

  // Desired inertia for translational and rotational axes.
  VectorXd Md_pos = VectorXd::Ones(3) * 10.0;
  VectorXd Md_rot = VectorXd::Ones(3) * 0.2;
  MatrixXd Md = MatrixXd::Zero(6, 6);
  Md.block<3, 3>(0, 0) = Md_pos.asDiagonal();
  Md.block<3, 3>(3, 3) = Md_rot.asDiagonal();

  // Critical damping: D = 2 * sqrt(M * K)
  const double Kd_pos    = 2.0 * std::sqrt(Md_pos[0] * Kp_pos);
  const double Kd_orient = 2.0 * std::sqrt(Md_rot[0] * Kp_orient);

  MatrixXd Kd = MatrixXd::Zero(6, 6);
  Kd.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3) * Kd_pos;
  Kd.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3) * Kd_orient;

  // Initialize UR Robot through HAL
  URRobot robot(robot_ip, freq);

  // External forces/torques at the end-effector (zero - free-space motion)
  Vector<double, 6> he = VectorXd::Zero(6);
  // Desired contact wrench (zero - purely motion tracking)
  VectorXd h_d_e = VectorXd::Zero(6);

  // --- End-effector wrench filtering ---
  // First-order low-pass filter to suppress force/torque sensor noise.
  //   he_filt = a * he_raw + (1 - a) * he_filt_prev,   a = 1 - exp(-2*pi*fc*dt)
  const double ft_cutoff_freq = 10.0;  // Hz
  const double ft_lpf_alpha   = 1.0 - std::exp(-2.0 * M_PI * ft_cutoff_freq * dt);
  Vector<double, 6> he_filtered = VectorXd::Zero(6);
  // Deadband half-widths: ignore forces below ft_force_deadband and torques below ft_torque_deadband.
  const double ft_force_deadband  = 1.5;  // N
  const double ft_torque_deadband = 1.0;  // Nm

  VectorXd u_max(DOF);
  // UR5e max and min torque
  u_max << 150.0, 150.0, 150.0, 28.0, 28.0, 28.0;
  VectorXd u_min = -u_max;

  controllers::ImpedanceController impedance_controller(Kp, Kd, Md, robot_model, controllers::ImpedanceController::OrientationRepresentation::QUATERNION);

  VectorXd x_d(DOF);
  VectorXd dx_d(DOF);
  VectorXd ddx_d(DOF);

  VectorXd q_init(DOF);
  q_init << 0.0, -pi/2, -pi/2, -pi/2, pi/2, 0.0;

  // Move robot to init position
  robot.move_joints(q_init);

  // Derive circle centre from initial end-effector pose
  Pose actual_robot_pose = robot.get_cartesian_tcp_pose();
  Vector3d center = actual_robot_pose.get_position();
  center[0] -= radius;   // shift so trajectory starts at robot's initial position

  // Desired orientation from the same model frame used by the controller.
  Matrix4d T_init = robot_model->get_fk_solver().forward_kinematics(robot.get_joint_positions());
  Quaterniond quat_d(T_init.topLeftCorner<3, 3>());
  quat_d.normalize();
  // Pack as Vector4d [w, x, y, z]
  Vector4d quat_d_vec(quat_d.w(), quat_d.x(), quat_d.y(), quat_d.z());

  signal(SIGINT, raise_flag);

  // Zero the force-torque sensor before starting the control loop
  std::this_thread::sleep_for(200ms);
  robot.zero_ft_sensor();
  std::this_thread::sleep_for(200ms);

  robot.set_control_mode(URRobot::ControlMode::TORQUE);
  robot.start_control();
  robot.step();

  // -------------------------------------------------------------------------
  // Control loop
  // -------------------------------------------------------------------------
  for (size_t step = 0; step < steps; ++step)
  {
    if (!flag_loop)
    {
      robot.stop_control();
      // Move robot to current joint position to end torque mode.
      robot.move_joints(robot.get_joint_positions());
      output_filestream.close();
      break;
    }

    // Start time
    steady_clock::time_point start_time = robot.init_period();
    const double t = step * dt;

    // Desired Cartesian pose, velocity and acceleration
    Vector3d pos_d, dpos_d, ddpos_d;
    circle_trajectory(center, radius, omega, t, ramp_time, pos_d, dpos_d, ddpos_d);

    x_d   << pos_d,    Vector3d::Zero();   // orientation part unused in QUATERNION mode
    dx_d  << dpos_d,   Vector3d::Zero();
    ddx_d << ddpos_d,  Vector3d::Zero();

    // Measured state
    VectorXd q_meas  = robot.get_joint_positions();
    VectorXd dq_meas = robot.get_joint_velocities();

    // Measured external wrench at the end-effector
    Vector<double, 6> he_raw = utils::std_vector_to_eigen(robot.get_tcp_forces());

    // Low-pass filter to attenuate sensor noise.
    he_filtered = ft_lpf_alpha * he_raw + (1.0 - ft_lpf_alpha) * he_filtered;

    // Deadband to reject small residual forces (N) / torques (Nm).
    he = apply_deadband(he_filtered, ft_force_deadband, ft_torque_deadband);

    // Impedance controller step (QUATERNION mode)
    impedance_controller.step(x_d, dx_d, ddx_d, q_meas, dq_meas, h_d_e, quat_d_vec);
    VectorXd y = impedance_controller.get_output();   // joint accelerations

    // Convert joint accelerations to joint torques via inverse dynamics
    // get gravity is substracted to avoid double-counting gravity in the direct_torque command.
    VectorXd tau = robot_model->inverse_dynamics(q_meas, dq_meas, y, he);
    VectorXd tau_g = robot_model->get_gravity(q_meas);
    tau -= tau_g;

    // Clamp the torques if they exceed max
    VectorXd tau_clamped = clamp_array(tau, u_max);

    // Collect actual end-effector position for logging
    Pose actual_robot_pose = robot.get_cartesian_tcp_pose();
    Vector3d pos_actual = actual_robot_pose.get_position();
    Vector3d rotvec_actual = actual_robot_pose.to_angle_axis_vector();

    // Write: time | tau (6) | q (6) | dq (6) | currents (6) | he (6) | pos (3) | rotvec (3) | pos_d (3) | y/ddq (6) | tau_clamped (6)
    VectorXd row(1 + DOF + DOF + DOF + DOF + DOF + 3 + 3 + 3 + DOF + DOF);
    row << t, tau + tau_g, q_meas, dq_meas, robot.get_actual_joint_currents(), he, pos_actual, rotvec_actual, pos_d, y, tau_clamped;
    csv_writer << eigen_to_std_vector(row);

    //VectorXd tau_zero(6);
    //tau_zero << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // Set joint torque reference
    robot.set_joint_torque_ref(tau_clamped);

    // Update the robot control
    robot.step();

    robot.wait_period(start_time);
  }

  robot.stop_control();
  // Move robot to current joint position to end torque mode.
  robot.move_joints(robot.get_joint_positions());
  output_filestream.close();
}
