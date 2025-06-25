#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sdu_controllers/controllers/force_control_inner_velocity_loop.hpp>
#include <sdu_controllers/math/forward_dynamics.hpp>
#include <sdu_controllers/math/inverse_dynamics_joint_space.hpp>
#include <sdu_controllers/kinematics/forward_kinematics.hpp>
#include <sdu_controllers/models/breeding_blanket_handling_robot_model.hpp>
#include <sdu_controllers/safety/safety_verifier.hpp>
#include <sdu_controllers/utils/utility.hpp>

#include <sdu_controllers/kinematics/forward_kinematics.hpp>

using namespace csv;
using namespace Eigen;
using namespace sdu_controllers;
using namespace sdu_controllers::utils;

int main()
{
  // Setup writing of output trajectory to csv.
  std::ofstream output_filestream;
  output_filestream.open("output.csv");
  auto csv_writer = make_csv_writer(output_filestream);
  std::vector<std::string> labels = {"timestamp",
    "q0", "q1", "q2", "q3", "q4", "q5", "q6",
    "pos0", "pos1", "pos2", "rpy0", "rpy1", "rpy2",
    "force0", "force1", "force2", "force3", "force4", "force5",
    "desired_force0", "desired_force1", "desired_force2",
    "desired_force3", "desired_force4", "desired_force5", };
  csv_writer << labels;

  // Initialize robot model and parameters
  auto robot_model = std::make_shared<models::BreedingBlanketHandlingRobotModel>();
  double freq = 500.0;
  double dt = 1.0 / freq;

  double Kp_pos_value = 1;
  double Kp_orient_value = 1;
  double Kd_pos_value = 100; // 2*sqrt(Kp_pos_value);
  double Kd_orient_value = 10; // 2*sqrt(Kp_orient_value);
  uint16_t ROBOT_DOF = robot_model->get_dof();

  VectorXd Kp_pos_vec = VectorXd::Ones(3) * Kp_pos_value;
  VectorXd Kp_orient_vec = VectorXd::Ones(3) * Kp_orient_value;
  VectorXd Kd_pos_vec = VectorXd::Ones(3) * Kd_pos_value;
  VectorXd Kd_orient_vec = VectorXd::Ones(3) * Kd_orient_value;

  MatrixXd Kp = MatrixXd::Zero(6, 6);
  Kp.setIdentity();
  Kp.block<3, 3>(0,0) = Kp_pos_vec.asDiagonal();
  Kp.block<3, 3>(3,3) = Kp_orient_vec.asDiagonal();
  MatrixXd Kd = MatrixXd::Zero(6, 6);
  Kd.setIdentity();
  Kd.block<3, 3>(0,0) = Kd_pos_vec.asDiagonal();
  Kd.block<3, 3>(3,3) = Kd_orient_vec.asDiagonal();

  VectorXd Md_pos = VectorXd::Ones(3) * 1;
  VectorXd Md_rot = VectorXd::Ones(3) * 1;
  MatrixXd Md = MatrixXd::Zero(6, 6);
  Md.setIdentity();
  Md.block<3, 3>(0, 0) = Md_pos.asDiagonal();
  Md.block<3, 3>(3, 3) = Md_rot.asDiagonal();

  VectorXd Kf_pos = VectorXd::Ones(3) * 1;
  VectorXd Kf_rot = VectorXd::Ones(3) * 1;
  MatrixXd Kf = MatrixXd::Zero(6, 6);
  Kf.setIdentity();
  Kf.block<3, 3>(0, 0) = Kf_pos.asDiagonal();
  Kf.block<3, 3>(3, 3) = Kf_rot.asDiagonal();

  controllers::ForceControlInnerVelocityLoop controller(Kp, Kd, Md, Kf, robot_model);
  math::InverseDynamicsJointSpace inv_dyn_jnt_space(robot_model);
  math::ForwardDynamics fwd_dyn(robot_model);

  Vector<double, 6> fd, fe, dfd;
  fd << 0, 0, 0,
        VectorXd::Ones(3) * 0;

  double Kenv_el = 100;
  VectorXd Kenv_vec = VectorXd::Ones(6) * Kenv_el;
  MatrixXd Kenv = Kenv_vec.asDiagonal();
  // double Denv = 20;

  VectorXd q(ROBOT_DOF);
  VectorXd dq(ROBOT_DOF);
  q << 0.0, 0., 3., 0., 0., 0.0, 0.0;
  dq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  Matrix4d T;
  VectorXd pos;
  Vector<double, 6> xe, xr;

  T = kinematics::forward_kinematics(q, robot_model);
  xr << T.block<3, 1>(0, 3),
        VectorXd::Zero(3);

  // Get safety bounds of robot.
  std::pair<Eigen::VectorXd, Eigen::VectorXd> joint_pos_bounds = robot_model->get_joint_pos_bounds();
  std::pair<Eigen::VectorXd, Eigen::VectorXd> joint_vel_bounds = robot_model->get_joint_vel_bounds();
  std::pair<Eigen::VectorXd, Eigen::VectorXd> joint_acc_bounds = robot_model->get_joint_acc_bounds();
  std::pair<Eigen::VectorXd, Eigen::VectorXd> joint_torque_bounds = robot_model->get_joint_torque_bounds();

  // Control loop
  double t_end = 100; //dt * 5;

  for (size_t j = 0; j < t_end / dt; j++)  // (const std::vector<double>& trajectory_point : input_trajectory)
  {
    std::cout << j * dt << std::endl;

    // dfd << 0, 0, 0.1 * dt, VectorXd::Zero(3);
    dfd << 0,
           (M_PI*cos((M_PI * j * dt)/50))/5 * dt,
           2 * M_PI *cos((M_PI * j * dt)/50) * dt,
           VectorXd::Zero(3);

    fd += dfd;

    // Add noise to q and dq
    VectorXd q_meas = q;
    VectorXd dq_meas = dq;
    //add_noise_to_vector(q_meas, 0.0, 0.001);
    //add_noise_to_vector(dq_meas, 0.0, 0.001);

    T = kinematics::forward_kinematics(q_meas, robot_model);
    pos = T.block<3, 1>(0, 3);
    xe << pos,
          VectorXd::Zero(3);

    fe = Kenv * (xe - xr);

    // Controller
    controller.step(fd, fe, q_meas, dq_meas);
    VectorXd y = controller.get_output();
    std::cout << "y: " << y << std::endl;
    VectorXd tau = inv_dyn_jnt_space.inverse_dynamics(y, q_meas, dq_meas);
    std::cout << "tau: " << tau << std::endl;

    // Simulation
    VectorXd ddq = fwd_dyn.forward_dynamics(q, dq, tau);
    std::cout << "ddq: " << ddq << std::endl;

    // Check bounds
    for (size_t i = 0; i < ROBOT_DOF; ++i)
    {
      if (ddq[i] < joint_acc_bounds.first[i])
        ddq[i] = joint_acc_bounds.first[i];
      else if (ddq[i] > joint_acc_bounds.second[i])
        ddq[i] = joint_acc_bounds.second[i];
    }

    // integrate to get velocity
    dq += ddq * dt;

    for (size_t i = 0; i < ROBOT_DOF; ++i)
    {
      if (dq[i] < joint_vel_bounds.first[i])
        dq[i] = joint_vel_bounds.first[i];
      else if (dq[i] > joint_vel_bounds.second[i])
        dq[i] = joint_vel_bounds.second[i];
    }

    // integrate to get position
    q += dq * dt;

    for (size_t i = 0; i < ROBOT_DOF; ++i)
    {
      if (q[i] < joint_pos_bounds.first[i])
        q[i] = joint_pos_bounds.first[i];
      else if (q[i] > joint_pos_bounds.second[i])
        q[i] = joint_pos_bounds.second[i];
    }

    // std::cout << "q:" << q << std::endl;
    MatrixXd T = kinematics::forward_kinematics(q, robot_model);
    VectorXd pos = T.block<3, 1>(0, 3);
    // std::cout << "pos:" << pos << std::endl;
    Matrix3d rot_mat = T.topLeftCorner(3, 3);
    Vector3d rpy_zyz = rot_mat.eulerAngles(2, 1, 2); // ZYZ representation
    // std::cout << "rpy_zyz:" << rpy_zyz << std::endl;
    VectorXd temp(1 + q.size()+pos.size()+rpy_zyz.size() + fe.size() + fd.size());
    temp << j * dt, q, pos, rpy_zyz, fe, fd;
    csv_writer << eigen_to_std_vector(temp);

    // std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  output_filestream.close();
}
