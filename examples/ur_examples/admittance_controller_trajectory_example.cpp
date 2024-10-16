#include <iostream>
#include <Eigen/Dense>
// #include <math.h>
// #include <sdu_controllers/math/math.hpp>
#include <sdu_controllers/controllers/admittance_controller_position.hpp>

#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>

#include <chrono>
#include <thread>
#include <csignal>

using namespace ur_rtde;
using namespace std; 
using namespace Eigen;
using namespace std::chrono;
using namespace sdu_controllers;

// Interrupt flag
bool flag_loop = true;
void raiseFlag(int)
{
  flag_loop = false;
}

// Type conversions
Affine3d pos_rotvec_to_T(const Vector3d &position, const AngleAxisd &rotation)
{
  Affine3d T = Affine3d::Identity(); 
  T.translation() = position;
  T.linear() = rotation.toRotationMatrix();
  return T;
}

Affine3d pos_rotmat_to_T(const Vector3d &position, const Matrix3d &rotation)
{
  Affine3d T = Affine3d::Identity(); 
  T.translation() = position;
  T.linear() = rotation;
  return T;
}

Affine3d pos_quat_to_T(const VectorXd pose)
{
  Affine3d T = Affine3d::Identity(); 
  T.translation() = pose.block<3, 1>(0, 0);
  T.linear() = Quaterniond(pose[3], pose[4], pose[5], pose[6]).toRotationMatrix();
  return T;
}

Affine3d stdvec_to_T(const std::vector<double> &pose)
{
  Vector3d position(pose[0], pose[1], pose[2]);

  Vector3d compact_eaa(pose[3], pose[4], pose[5]);
  double angle = compact_eaa.norm(); 

  Affine3d T = pos_rotvec_to_T(position, AngleAxisd(angle, compact_eaa.normalized())); 
  return T;
}

std::vector<double> T_to_stdvec(const Affine3d &T)
{
  std::vector<double> pose; 
  for(double p: T.translation())
  {
    pose.push_back(p);
  }

  AngleAxisd eaa(T.linear());
  Vector3d compact_eaa(eaa.angle() * eaa.axis());
  for(double p: compact_eaa)
  {
    pose.push_back(p);
  }

  return pose;
}


template<class Derived>
  inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> &vec)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1],
            vec[2], 0.0, -vec[0],
            -vec[1], vec[0], 0.0)
        .finished();
  }


MatrixXd adjoint(const Affine3d &T)
{
  MatrixXd adj = MatrixXd::Zero(6, 6);
  adj.block<3,3>(0,0) = T.linear();
  adj.block<3,3>(3,0) = skew(T.translation()) * T.linear();
  adj.block<3,3>(3,3) = T.linear();
  return adj;
}


VectorXd wrench_trans(const Vector3d &torques, const Vector3d &forces, const Affine3d &T)
{
  VectorXd wrench_in_A(6); 
  wrench_in_A << torques[0], torques[1], torques[2], forces[0], forces[1], forces[2];
  VectorXd wrench_in_B = adjoint(T).transpose() * wrench_in_A;
  return wrench_in_B; 
}

Vector3d get_circle_target(const Vector3d &pose, double timestep, double radius=0.075, double freq=0.5)
{
  Vector3d circle_target;
  circle_target[0] = pose[0] + radius * cos(2 * M_PI * freq * timestep); 
  circle_target[1] = pose[1] + radius * sin(2 * M_PI * freq * timestep);
  circle_target[2] = pose[2];
  return circle_target;
}



int main(int argc, char* argv[])
{
  double frequency = 500.0;
  double dt = 1./frequency;

  double counter = 0.0; 

  // Initialize admittance control
  VectorXd u;
  controllers::AdmittanceControllerPosition adm_controller;

  string robot_ip = "127.0.0.1";
  if (argc > 1)
  {
    robot_ip = argv[1];
  }

  RTDEReceiveInterface rtde_receive(robot_ip); 
  RTDEControlInterface rtde_control(robot_ip);

  rtde_control.moveL({0.05, -0.5, 0.3, 0., 3.141, 0.}, 0.5, 0.2);
  rtde_control.zeroFtSensor(); 
  std::this_thread::sleep_for(200ms);

  Affine3d T_base_tcp = stdvec_to_T(rtde_receive.getActualTCPPose());

  // Define tip 
  Affine3d T_tcp_tip = pos_rotvec_to_T(Vector3d(0, 0, 0.194), AngleAxisd(0., Vector3d::UnitZ()));
  Affine3d T_tip_tcp = T_tcp_tip.inverse(); 
  Affine3d T_base_tip = T_base_tcp * T_tcp_tip; 

  Affine3d T_base_tip_init = T_base_tip;

  // Set target circle
  Vector3d x_desired = get_circle_target(T_base_tip.translation(), counter); 
  Affine3d T_base_tip_circle = pos_rotmat_to_T(x_desired, T_base_tip.linear());

  Affine3d T_base_tcp_circle = T_base_tip_circle * T_tip_tcp; 

  std::vector<double> robot_pose = T_to_stdvec(T_base_tcp_circle);
  rtde_control.moveL(robot_pose);

  signal(SIGINT, raiseFlag);

  try
  {
    while (flag_loop)
    {
        // Start time
        steady_clock::time_point start_time = rtde_control.initPeriod();
        
        // Get current position
        T_base_tcp = stdvec_to_T(rtde_receive.getActualTCPPose());
        std::vector<double> ft = rtde_receive.getActualTCPForce();

        // Transform into compliant coordinate system (tip?)
        T_base_tip = T_base_tcp * T_tcp_tip;

        // Get current force & torque
        Vector3d f_base(ft[0], ft[1], ft[2]);
        Vector3d mu_base(ft[3], ft[4], ft[5]);

        // Rotate forces from base frame into TCP
        Matrix3d R_tcp_base = T_base_tcp.linear().inverse();
        Vector3d f_tcp = R_tcp_base * f_base;
        Vector3d mu_tcp = R_tcp_base * mu_base;

        // use wrench transform to place the force torque in the tip.
        VectorXd ft_tip = wrench_trans(mu_tcp, f_tcp, T_tcp_tip);

        // Rotate forces back to base frame
        Vector3d f_base_tip = T_base_tip.linear() * ft_tip.block<3,1>(3,0);

        // Get circle target
        x_desired = get_circle_target(T_base_tip_init.translation(), counter);

        // Step controller
        adm_controller.step(f_base_tip, ft_tip.block<3,1>(0,0), x_desired, Quaterniond(T_base_tip_init.linear()));
        u = adm_controller.get_output();

        // Rotate output from tip to TCP before sending it to the robot
        Affine3d T_base_tip_out = pos_quat_to_T(u);
        Affine3d T_base_tcp_out = T_base_tip_out * T_tip_tcp;

        // Send control value
        robot_pose = T_to_stdvec(T_base_tcp_out);
        rtde_control.servoL(robot_pose, 0.0, 0.0, dt, 0.03, 2000);
        rtde_control.waitPeriod(start_time);

        counter = counter + dt;
    }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

  // Shutdown
  rtde_control.servoStop(10.0);

  rtde_receive.disconnect();
  rtde_control.disconnect();
  return 0;
}