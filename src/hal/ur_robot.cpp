#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <chrono>
#include <sdu_controllers/hal/ur_robot.hpp>
#include <sdu_controllers/utils/utility.hpp>

#include <stdexcept>
#include <vector>

namespace sdu_controllers::hal
{
  URRobot::URRobot(const std::string& ip, double control_frequency)
  {
    rtde_receive_ = std::make_shared<ur_rtde::RTDEReceiveInterface>(ip, control_frequency);
    rtde_control_ = std::make_shared<ur_rtde::RTDEControlInterface>(ip, control_frequency);
    control_frequency_ = control_frequency;
    dt_ = 1.0 / control_frequency_;
    control_mode_ = ControlMode::UNDEFINED;
    curr_state_ = ControlStates::INIT;
    prev_state_ = ControlStates::NONE;
    start_control_ = false;
    stop_control_ = false;

    // servo options
    servo_vel_ = 0.0; // Not used currently
    servo_acc_ = 0.0; // Not used currently
    servo_p_gain_ = 2000;  // proportional gain
    servo_lookahead_t_ = 0.03;  // lookahead time

    deceleration_rate_ = 10.0; // m/s^2
    vel_tool_acceleration_ = 35.0;  // 1.4  # m/s^2
  }

  void URRobot::step()
  {
    // State-machine control logic
    switch (curr_state_)
    {
      case ControlStates::NONE:
        break;

      case ControlStates::INIT:
        if(prev_state_ != curr_state_)
        {
          if (control_mode_ == ControlMode::JOINT_POSITION)
          {
            // Initialize joint_pos_ref_ to current joint position of the robot, this is to avoid any sudden jumps.
            joint_pos_ref_ = utils::std_vector_to_eigen(rtde_receive_->getActualQ());
          }
          else if (control_mode_ == ControlMode::CARTESIAN)
          {
            // Initialize cartesian_pos_ref_ to current pose of the robot, this is to avoid any sudden jumps.
            cartesian_pose_ref_ = math::Pose(rtde_receive_->getActualTCPPose());
          }
          else if (control_mode_ == ControlMode::JOINT_VELOCITY)
            joint_vel_ref_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          else if (control_mode_ == ControlMode::CARTESIAN_VELOCITY)
            cartesian_vel_ref_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

          // This code will run once on entry to state
          prev_state_ = curr_state_;
        }
        // Init complete, goto STOPPED state
        curr_state_ = ControlStates::STOPPED;
        break;

      case ControlStates::STOPPED:
        if(prev_state_ != curr_state_)
        {
          // This code will run once on entry to state
          prev_state_ = curr_state_;
        }
        // Check if the control should be started. (The user might want to perform safety checks before triggering the running state)
        if (start_control_)
          curr_state_ = ControlStates::RUNNING;
        break;

      case ControlStates::RUNNING:
        if(prev_state_ != curr_state_)
        {
          // This code will run once on entry to state
          prev_state_ = curr_state_;
        }
        // Check if the control should be stopped.
        if (stop_control_)
        {
          if (control_mode_ == ControlMode::JOINT_POSITION || control_mode_ == ControlMode::CARTESIAN)
            rtde_control_->servoStop();
          else if (control_mode_ == ControlMode::JOINT_VELOCITY || control_mode_ == ControlMode::CARTESIAN_VELOCITY)
            rtde_control_->speedStop();
          curr_state_ = ControlStates::STOPPED;
          break;
        }

        // Perform control
        if (control_mode_ == ControlMode::JOINT_POSITION)
        {
          std::vector<double> q_vec(joint_pos_ref_.begin(), joint_pos_ref_.end());
          rtde_control_->servoJ(q_vec, servo_vel_, servo_acc_, dt_, servo_lookahead_t_, servo_p_gain_);
        }
        else if (control_mode_ == ControlMode::CARTESIAN)
        {
          Eigen::Vector3d pos = cartesian_pose_ref_.get_position();
          Eigen::Vector3d rotvec = cartesian_pose_ref_.to_angle_axis_vector();
          std::vector<double> pose_rotvec{pos[0], pos[1], pos[2], rotvec[0], rotvec[1], rotvec[2]};
          rtde_control_->servoL(pose_rotvec, servo_vel_, servo_acc_, dt_, servo_lookahead_t_, servo_p_gain_);
        }
        else if (control_mode_ == ControlMode::UNDEFINED)
        {
          std::cerr << "Undefined control mode specified transitioning to ERROR state." << std::endl;
          curr_state_ = ControlStates::ERROR;
        }

        break;

      case ControlStates::ERROR:
        if(prev_state_ != curr_state_)
        {
          // This code will run once on entry to state
          prev_state_ = curr_state_;
        }
        break;

      default:
        break;
    }
  }

  bool URRobot::start_control()
  {
    start_control_ = true;
    return true;
  }

  bool URRobot::stop_control()
  {
    stop_control_ = true;
    return true;
  }

  std::chrono::steady_clock::time_point URRobot::init_period()
  {
    return rtde_control_->initPeriod();
  }

  void URRobot::wait_period(const std::chrono::steady_clock::time_point &t_cycle_start)
  {
    rtde_control_->waitPeriod(t_cycle_start);
  }

  bool URRobot::set_control_mode(ControlMode control_mode)
  {
    control_mode_ = control_mode;
    return true;
  }

  bool URRobot::set_joint_pos_ref(const Eigen::Vector<double, ROBOT_DOF>& q)
  {
    // Check if the control mode has been set, otherwise set it to JOINT_POSITION.
    if (control_mode_ == ControlMode::UNDEFINED)
      set_control_mode(ControlMode::JOINT_POSITION);

    joint_pos_ref_ = q;
    return true;
  }

  bool URRobot::set_cartesian_pose_ref(const math::Pose& pose)
  {
    // Check if the control mode has been set, otherwise set it to CARTESIAN_POSITION.
    if (control_mode_ == ControlMode::UNDEFINED)
      set_control_mode(ControlMode::CARTESIAN);

    cartesian_pose_ref_ = pose;
    return true;
  }

  bool URRobot::set_joint_vel_ref(const Eigen::Vector<double, ROBOT_DOF> &dq, double acceleration)
  {
    // Check if the control mode has been set, otherwise set it to JOINT_VELOCITY.
    if (control_mode_ == ControlMode::UNDEFINED)
      set_control_mode(ControlMode::JOINT_VELOCITY);

    joint_vel_ref_ = dq;
    return true;
  }

  bool URRobot::set_cartesian_vel_ref(const Eigen::Vector<double, ROBOT_DOF> &xd, double acceleration)
  {
    // Check if the control mode has been set, otherwise set it to CARTESIAN_VELOCITY.
    if (control_mode_ == ControlMode::UNDEFINED)
      set_control_mode(ControlMode::CARTESIAN_VELOCITY);

    cartesian_vel_ref_ = xd;
    return true;
  }

  void URRobot::zero_ft_sensor()
  {
    rtde_control_->zeroFtSensor();
  }

  bool URRobot::move_joints(const Eigen::Vector<double, ROBOT_DOF>& q, double velocity, double acceleration, bool asynchronous)
  {
    std::vector<double> q_vec(q.begin(), q.end());
    return rtde_control_->moveJ(q_vec, velocity, acceleration, asynchronous);
  }

  bool URRobot::move_cartesian(const math::Pose& pose, double velocity, double acceleration, bool asynchronous)
  {
    Eigen::Vector3d pos = pose.get_position();
    Eigen::Vector3d rotvec = pose.to_angle_axis_vector();
    std::vector<double> pose_rotvec{pos[0], pos[1], pos[2], rotvec[0], rotvec[1], rotvec[2]};
    return rtde_control_->moveL(pose_rotvec, velocity, acceleration, asynchronous);
  }

  Eigen::VectorXd URRobot::get_joint_torques()
  {
    Eigen::VectorXd torques = utils::std_vector_to_eigen(rtde_control_->getJointTorques());
    return torques;
  }

  Eigen::VectorXd URRobot::get_joint_positions()
  {
    Eigen::VectorXd q = utils::std_vector_to_eigen(rtde_receive_->getActualQ());
    return q;
  }

  Eigen::VectorXd URRobot::get_joint_velocities()
  {
    Eigen::VectorXd qd = utils::std_vector_to_eigen(rtde_receive_->getActualQd());
    return qd;
  }

  math::Pose URRobot::get_cartesian_tcp_pose()
  {
    return math::Pose(rtde_receive_->getActualTCPPose());
  }

  std::vector<double> URRobot::get_tcp_forces()
  {
    return rtde_receive_->getActualTCPForce();
  }

};  // namespace sdu_controllers::hal
