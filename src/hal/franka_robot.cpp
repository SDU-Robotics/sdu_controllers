#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <Eigen/Dense>
#include <array>
#include <sdu_controllers/hal/franka_robot.hpp>
#include <sdu_controllers/utils/utility.hpp>
#include <vector>

namespace sdu_controllers::hal
{

  FrankaRobot::FrankaRobot(const std::string& ip, double control_frequency) : robot_(ip), robot_model_(robot_.loadModel())
  {
    control_frequency_ = control_frequency;
    dt_ = 1.0 / control_frequency_;
    control_mode_ = ControlMode::UNDEFINED;
    curr_state_ = ControlStates::INIT;
    prev_state_ = ControlStates::NONE;
    start_control_ = false;
    stop_control_ = false;

    // Set a default collision behavior, joint impedance and cartesian impedance.
    set_default_behavior();

    // Read and initialize the current robot state
    robot_state_ = robot_.readOnce();
  }

  void FrankaRobot::set_default_behavior()
  {
    robot_.setCollisionBehavior(
        { { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 } },
        { { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 } },
        { { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 } },
        { { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 } },
        { { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 } },
        { { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 } },
        { { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 } },
        { { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 } });
    robot_.setJointImpedance({ { 3000, 3000, 3000, 2500, 2500, 2000, 2000 } });
    robot_.setCartesianImpedance({ { 3000, 3000, 3000, 300, 300, 300 } });
  }

  franka::JointPositions FrankaRobot::joint_position_control_cb(const franka::RobotState& state, franka::Duration /*period*/)
  {
    // Update the robot state
    robot_state_ = state;
    std::array<double, ROBOT_DOF> q_arr;
    std::move(joint_pos_ref_.begin(), joint_pos_ref_.end(), q_arr.begin());
    return franka::JointPositions(q_arr);
  }

  franka::CartesianPose FrankaRobot::cartesian_pose_control_cb(const franka::RobotState& state, franka::Duration /*period*/)
  {
    // Update the robot state
    robot_state_ = state;
    Eigen::Affine3d cart_pose = cartesian_pose_ref_.to_transform();
    std::array<double, 16> array;
    std::copy(cart_pose.data(), cart_pose.data() + array.size(), array.begin());
    return { array };
  }

  void FrankaRobot::step()
  {
    // State-machine control logic
    switch (curr_state_)
    {
      case ControlStates::NONE: break;

      case ControlStates::INIT:
        if (prev_state_ != curr_state_)
        {
          if (control_mode_ == ControlMode::JOINT_POSITION)
          {
            // Initialize joint_pos_ref_ to current joint position of the robot, this is to avoid any sudden jumps.
            joint_pos_ref_ = utils::std_array_to_eigen_vector(robot_state_.q);
          }
          else if (control_mode_ == ControlMode::CARTESIAN_POSE)
          {
            // Initialize cartesian_pos_ref_ to current pose of the robot, this is to avoid any sudden jumps.
            Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state_.O_T_EE.data()));
            cartesian_pose_ref_ = math::Pose(initial_transform);
          }
          else if (control_mode_ == ControlMode::JOINT_VELOCITY)
            joint_vel_ref_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
          else if (control_mode_ == ControlMode::CARTESIAN_VELOCITY)
            cartesian_vel_ref_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

          // This code will run once on entry to state
          prev_state_ = curr_state_;
        }
        // Init complete, goto STOPPED state
        curr_state_ = ControlStates::STOPPED;
        break;

      case ControlStates::STOPPED:
        if (prev_state_ != curr_state_)
        {
          // This code will run once on entry to state
          prev_state_ = curr_state_;
        }
        // Check if the control should be started. (The user might want to perform safety checks before triggering the
        // running state)
        if (start_control_)
        {
          try
          {
            if (control_mode_ == ControlMode::JOINT_POSITION)
            {
              robot_.control(
                  std::bind(&FrankaRobot::joint_position_control_cb, this, std::placeholders::_1, std::placeholders::_2));
            }
            else if (control_mode_ == ControlMode::CARTESIAN_POSE)
            {
              robot_.control(
                  std::bind(&FrankaRobot::cartesian_pose_control_cb, this, std::placeholders::_1, std::placeholders::_2));
            }
            curr_state_ = ControlStates::RUNNING;
          }
          catch (const franka::Exception& e)
          {
            std::cerr << "Failed to execute motion: " << e.what() << std::endl;
            curr_state_ = ControlStates::ERROR;
            break;
          }
        }
        break;

      case ControlStates::RUNNING:
        if (prev_state_ != curr_state_)
        {
          // This code will run once on entry to state
          prev_state_ = curr_state_;
        }
        // Check if the control should be stopped.
        if (stop_control_)
        {
          // TODO: Check if torques should be zeroed before stopping in the case of TORQUE control.
          robot_.stop();
          curr_state_ = ControlStates::STOPPED;
          break;
        }

        // Perform control
        if (control_mode_ == ControlMode::JOINT_POSITION)
        {
        }
        else if (control_mode_ == ControlMode::CARTESIAN_POSE)
        {
        }
        else if (control_mode_ == ControlMode::UNDEFINED)
        {
          std::cerr << "Undefined control mode specified transitioning to ERROR state." << std::endl;
          curr_state_ = ControlStates::ERROR;
        }

        break;

      case ControlStates::ERROR:
        if (prev_state_ != curr_state_)
        {
          // This code will run once on entry to state
          prev_state_ = curr_state_;
        }
        break;

      default: break;
    }
  }

  bool FrankaRobot::start_control()
  {
    start_control_ = true;
    return true;
  }

  bool FrankaRobot::stop_control()
  {
    stop_control_ = true;
    return true;
  }

  bool FrankaRobot::set_control_mode(ControlMode control_mode)
  {
    control_mode_ = control_mode;
    return true;
  }

  bool FrankaRobot::set_joint_pos_ref(const Eigen::Vector<double, ROBOT_DOF>& q)
  {
    // Check if the control mode has been set, otherwise set it to JOINT_POSITION.
    if (control_mode_ == ControlMode::UNDEFINED)
      set_control_mode(ControlMode::JOINT_POSITION);

    joint_pos_ref_ = q;
    return true;
  }

  bool FrankaRobot::set_cartesian_pose_ref(const math::Pose& pose)
  {
    // Check if the control mode has been set, otherwise set it to CARTESIAN_POSITION.
    if (control_mode_ == ControlMode::UNDEFINED)
      set_control_mode(ControlMode::CARTESIAN_POSE);

    cartesian_pose_ref_ = pose;
    return true;
  }

  bool FrankaRobot::set_joint_vel_ref(const Eigen::Vector<double, ROBOT_DOF>& dq, double acceleration)
  {
    // Check if the control mode has been set, otherwise set it to JOINT_VELOCITY.
    if (control_mode_ == ControlMode::UNDEFINED)
      set_control_mode(ControlMode::JOINT_VELOCITY);

    joint_vel_ref_ = dq;
    return true;
  }

  bool FrankaRobot::set_cartesian_vel_ref(const Eigen::Vector<double, ROBOT_DOF>& xd, double acceleration)
  {
    // Check if the control mode has been set, otherwise set it to CARTESIAN_VELOCITY.
    if (control_mode_ == ControlMode::UNDEFINED)
      set_control_mode(ControlMode::CARTESIAN_VELOCITY);

    cartesian_vel_ref_ = xd;
    return true;
  }

  bool FrankaRobot::move_joints(
      const Eigen::Vector<double, ROBOT_DOF>& q,
      double velocity,
      double acceleration,
      bool asynchronous)
  {
    // std::vector<double> q_vec(q.begin(), q.end());
    // return rtde_control_->moveJ(q_vec, velocity, acceleration, asynchronous);
  }

  bool FrankaRobot::move_cartesian(const math::Pose& pose, double velocity, double acceleration, bool asynchronous)
  {
    // Eigen::Vector3d pos = pose.get_position();
    // Eigen::Vector3d rotvec = pose.to_angle_axis_vector();
    // std::vector<double> pose_rotvec{ pos[0], pos[1], pos[2], rotvec[0], rotvec[1], rotvec[2] };
    // return rtde_control_->moveL(pose_rotvec, velocity, acceleration, asynchronous);
  }

  Eigen::VectorXd FrankaRobot::get_joint_torques()
  {
    Eigen::VectorXd torques = utils::std_array_to_eigen_vector(robot_state_.tau_J);
    return torques;
  }

  Eigen::VectorXd FrankaRobot::get_joint_positions()
  {
    Eigen::VectorXd q = utils::std_array_to_eigen_vector(robot_state_.q);
    return q;
  }

  Eigen::VectorXd FrankaRobot::get_joint_velocities()
  {
    Eigen::VectorXd dq = utils::std_array_to_eigen_vector(robot_state_.dq);
    return dq;
  }

  math::Pose FrankaRobot::get_cartesian_tcp_pose()
  {
    Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state_.O_T_EE.data()));
    return math::Pose(current_transform);
  }

  std::vector<double> FrankaRobot::get_tcp_forces()
  {
    std::array<double, 6> tcp_wrench = robot_state_.O_F_ext_hat_K;
    std::vector<double> tcp_wrench_vec(tcp_wrench.begin(), tcp_wrench.end());
    return tcp_wrench_vec;
  }

};  // namespace sdu_controllers::hal
