#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <Eigen/Dense>
#include <array>
#include <sdu_controllers/hal/franka_robot.hpp>
#include <sdu_controllers/hal/robot.hpp>
#include <sdu_controllers/math/pose.hpp>
#include <sdu_controllers/utils/utility.hpp>
#include <thread>
#include <vector>
#include <chrono>
#include <mutex>

namespace sdu_controllers::hal
{
  FrankaRobot::FrankaRobot(const std::string& ip, double control_frequency)
      : robot_(ip, franka::RealtimeConfig(franka::RealtimeConfig::kIgnore)),
        robot_model_(robot_.loadModel())
  {
    control_frequency_ = control_frequency;
    dt_ = 1.0 / control_frequency_;
    control_mode_ = ControlMode::UNDEFINED;
    curr_state_ = ControlStates::INIT;
    prev_state_ = ControlStates::NONE;
    start_control_ = false;
    stop_control_ = false;
    stop_receive_ = false;
    motion_finished_ = false;

    // Set a default collision behavior, joint impedance and cartesian impedance.
    set_default_behavior();

    // Read and initialize the current robot state
    robot_state_ = robot_.readOnce();
    //receive_thread_ = std::thread(&FrankaRobot::receive_robot_state, this);
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
    //robot_.setJointImpedance({ { 3000, 3000, 3000, 2500, 2500, 2000, 2000 } });
    //robot_.setCartesianImpedance({ { 300, 300, 300, 50, 50, 50 } });
  }

  void FrankaRobot::set_current_pose(const franka::CartesianPose& pose)
  {
    std::lock_guard<std::mutex> lk(update_pose_mutex_);
    current_pose_ = pose;
  }

  franka::CartesianPose FrankaRobot::get_current_pose()
  {
    std::lock_guard<std::mutex> lk(update_pose_mutex_);
    return current_pose_;
  }

  double FrankaRobot::get_step_time()
  {
    return step_time_;
  }

  void FrankaRobot::update_robot_state(const franka::RobotState &robot_state)
  {
    std::lock_guard<std::mutex> lk(robot_state_mutex_);
    robot_state_ = robot_state;
  }

  void FrankaRobot::receive_robot_state()
  {
    while (!stop_receive_)
    {
      if (curr_state_ == ControlStates::NONE || curr_state_ == ControlStates::INIT || curr_state_ == ControlStates::STOPPED)
      {
        update_robot_state(robot_.readOnce());
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  void FrankaRobot::control()
  {
    if (control_mode_ == ControlMode::TORQUE)
    {
      robot_.control(
          [this](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques
          {
            //update_robot_state(robot_state);
            std::array<double, 7> tau_d_array{};
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = joint_torque_ref_;
            franka::Torques torques(tau_d_array);
            torques.motion_finished = motion_finished_;
            return torques;
          });
    }
    else if (control_mode_ == ControlMode::JOINT_POSITION)
    {
      robot_.control(
          [this](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions
          {
            //update_robot_state(robot_state);
            std::array<double, ROBOT_DOF> q_arr;
            std::move(joint_pos_ref_.begin(), joint_pos_ref_.end(), q_arr.begin());
            franka::JointPositions joint_positions(q_arr);
            joint_positions.motion_finished = motion_finished_;
            return joint_positions;
          });
    }
    else if (control_mode_ == ControlMode::CARTESIAN_POSE)
    {
      //std::array<double, 16> initial_pose;
      //double time{0.0};
      robot_.control(
          [=](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose
          {
            step_time_ += period.toSec();

            if (step_time_ == 0.0)
            {
              set_current_pose(franka::CartesianPose(robot_state.O_T_EE_c));
            }

            /*constexpr double kRadius = 0.3;
            double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * step_time_));
            double delta_x = kRadius * std::sin(angle);
            double delta_z = kRadius * (std::cos(angle) - 1);

            std::array<double, 16> new_pose = initial_pose;
            new_pose[12] += delta_x;
            new_pose[14] += delta_z;*/

            franka::CartesianPose new_pose = get_current_pose();
            Eigen::Affine3d new_pose_transform(Eigen::Matrix4d::Map(new_pose.O_T_EE.data()));
            math::Pose my_new_pose(new_pose_transform);
            std::cout << "new_pose: " << my_new_pose.to_string() << std::endl;

            if (step_time_ >= 10.0) {
              std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
              return franka::MotionFinished(new_pose);
            }
            return new_pose;

            /*else
            {
              // The rest of your control loop
              Eigen::Affine3d current_pose_affine(Eigen::Matrix4d::Map(robot_state.O_T_EE_c.data())); // O_T_EE_c
              std::array<double, 16> array_current;
              std::copy(current_pose_affine.data(), current_pose_affine.data() + array_current.size(), array_current.begin());
              math::Pose current_pose(current_pose_affine);
              std::cout << "Current pose: " << current_pose.to_string() << std::endl;
              std::cout << "Sending: " << cartesian_pose_ref_.to_string() << std::endl;
              Eigen::Affine3d cart_pose = cartesian_pose_ref_.to_transform();
              std::array<double, 16> array;
              std::copy(cart_pose.data(), cart_pose.data() + array.size(), array.begin());
              franka::CartesianPose franka_cart_pose(robot_state.O_T_EE_c);
              franka_cart_pose.motion_finished = motion_finished_;
              return franka_cart_pose;
            }*/
          });
    }
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
            joint_vel_ref_ = Eigen::VectorXd::Zero(ROBOT_DOF);
          else if (control_mode_ == ControlMode::CARTESIAN_VELOCITY)
            cartesian_vel_ref_ << Eigen::VectorXd::Zero(ROBOT_DOF);
          else if (control_mode_ == ControlMode::TORQUE)
            joint_torque_ref_ << Eigen::VectorXd::Zero(ROBOT_DOF);

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
          // Stop receiving
          stop_receive_ = true;
          // join the control thread
          if (receive_thread_.joinable())
          {
            receive_thread_.join();
          }

          try
          {
            control_thread_ = std::thread(&FrankaRobot::control, this);
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

        if (control_mode_ == ControlMode::UNDEFINED)
        {
          std::cerr << "Undefined control mode specified transitioning to ERROR state." << std::endl;
          curr_state_ = ControlStates::ERROR;
          break;
        }
        // Notice! the control for the different ControlModes is done in the respective callback functions.

        // Check if the control should be stopped
        if (stop_control_)
        {
          if (control_mode_ == ControlMode::JOINT_POSITION || control_mode_ == ControlMode::CARTESIAN_POSE ||
              control_mode_ == ControlMode::TORQUE)
          {
            motion_finished_ = true;
          }

          //if (robot_state_.robot_mode == franka::RobotMode::kIdle)
          //{
          // Reset motion_finished and stop control and receive variable.
          //           motion_finished_ = false;
          //stop_control_ = false;
          //stop_receive_ = false;
          //
          robot_.stop();

          // join the control thread
          if (control_thread_.joinable())
          {
            control_thread_.join();
          }

          // Start receive thread again
          //receive_thread_ = std::thread(&FrankaRobot::receive_robot_state, this);

          curr_state_ = ControlStates::STOPPED;
          break;
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
    step();
    if (curr_state_ == ControlStates::RUNNING)
      return true;
    else
      return false;
  }

  bool FrankaRobot::stop_control()
  {
    stop_control_ = true;
    step();
    if (curr_state_ == ControlStates::STOPPED)
    {
      return true;
    }
    else
    {
      std::cerr << "Was unable to stop the control!" << std::endl;
      return false;
    }
  }

  bool FrankaRobot::set_control_mode(ControlMode control_mode)
  {
    control_mode_ = control_mode;
    step();
    if (curr_state_ == ControlStates::STOPPED)
      return true;
    else
      return false;
  }

  bool FrankaRobot::set_joint_torque_ref(const Eigen::Vector<double, ROBOT_DOF>& tau_d)
  {
    joint_torque_ref_ = tau_d;
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

  bool FrankaRobot::move_joints(const Eigen::Vector<double, ROBOT_DOF>& q, double speed_factor)
  {
    try
    {
      stop_receive_ = true;
      // join the control thread
      if (receive_thread_.joinable())
      {
        receive_thread_.join();
      }

      std::array<double, 7> q_goal;
      std::move(q.begin(), q.end(), q_goal.begin());
      MotionGenerator motion_generator(speed_factor, q_goal);
      robot_.control(motion_generator);

      stop_receive_ = false;
      //receive_thread_ = std::thread(&FrankaRobot::receive_robot_state, this);
    }
    catch (const franka::Exception& e)
    {
      std::cout << e.what() << std::endl;
      return false;
    }
    return true;
  }

  bool FrankaRobot::move_cartesian(const math::Pose& pose, double velocity, double acceleration, bool asynchronous)
  {
    // TODO: Implement simple cartesian movement (generate_cartesian_pose_motion.cpp in libfranka)

    return false;
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

  math::Pose FrankaRobot::get_actual_tcp_pose()
  {
    robot_state_ = robot_.readOnce();
    Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state_.O_T_EE.data()));
    return math::Pose(current_transform);
  }

  math::Pose FrankaRobot::get_target_tcp_pose()
  {
    Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state_.O_T_EE_c.data()));
    return math::Pose(current_transform);
  }

  std::vector<double> FrankaRobot::get_tcp_forces()
  {
    std::array<double, 6> tcp_wrench = robot_state_.O_F_ext_hat_K;
    std::vector<double> tcp_wrench_vec(tcp_wrench.begin(), tcp_wrench.end());
    return tcp_wrench_vec;
  }

  /*  Franka MotionGenerator */

  MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 7> q_goal) : q_goal_(q_goal.data())
  {
    dq_max_ *= speed_factor;
    ddq_max_start_ *= speed_factor;
    ddq_max_goal_ *= speed_factor;
    dq_max_sync_.setZero();
    q_start_.setZero();
    delta_q_.setZero();
    t_1_sync_.setZero();
    t_2_sync_.setZero();
    t_f_sync_.setZero();
    q_1_.setZero();
  }

  bool MotionGenerator::calculate_desired_values(double t, Vector7d* delta_q_d) const
  {
    Vector7i sign_delta_q;
    sign_delta_q << delta_q_.cwiseSign().cast<int>();
    Vector7d t_d = t_2_sync_ - t_1_sync_;
    Vector7d delta_t_2_sync = t_f_sync_ - t_2_sync_;
    std::array<bool, 7> joint_motion_finished{};

    for (size_t i = 0; i < 7; i++)
    {
      if (std::abs(delta_q_[i]) < k_delta_q_motion_finished)
      {
        (*delta_q_d)[i] = 0;
        joint_motion_finished[i] = true;
      }
      else
      {
        if (t < t_1_sync_[i])
        {
          (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
                            (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
        }
        else if (t >= t_1_sync_[i] && t < t_2_sync_[i])
        {
          (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
        }
        else if (t >= t_2_sync_[i] && t < t_f_sync_[i])
        {
          (*delta_q_d)[i] = delta_q_[i] + 0.5 *
                                              (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                                                   (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                                                   std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
                                               (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                                              dq_max_sync_[i] * sign_delta_q[i];
        }
        else
        {
          (*delta_q_d)[i] = delta_q_[i];
          joint_motion_finished[i] = true;
        }
      }
    }
    return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(), [](bool x) { return x; });
  }

  void MotionGenerator::calculate_synchronized_values()
  {
    Vector7d dq_max_reach(dq_max_);
    Vector7d t_f = Vector7d::Zero();
    Vector7d delta_t_2 = Vector7d::Zero();
    Vector7d t_1 = Vector7d::Zero();
    Vector7d delta_t_2_sync = Vector7d::Zero();
    Vector7i sign_delta_q;
    sign_delta_q << delta_q_.cwiseSign().cast<int>();

    for (size_t i = 0; i < 7; i++)
    {
      if (std::abs(delta_q_[i]) > k_delta_q_motion_finished)
      {
        if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                     3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i])))
        {
          dq_max_reach[i] = std::sqrt(
              4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] * (ddq_max_start_[i] * ddq_max_goal_[i]) /
              (ddq_max_start_[i] + ddq_max_goal_[i]));
        }
        t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
        delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
        t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
      }
    }
    double max_t_f = t_f.maxCoeff();
    for (size_t i = 0; i < 7; i++)
    {
      if (std::abs(delta_q_[i]) > k_delta_q_motion_finished)
      {
        double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
        double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
        double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
        double delta = b * b - 4.0 * a * c;
        if (delta < 0.0)
        {
          delta = 0.0;
        }
        dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
        t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
        delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
        t_f_sync_[i] = (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
        t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
        q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
      }
    }
  }

  franka::JointPositions MotionGenerator::operator()(const franka::RobotState& robot_state, franka::Duration period)
  {
    time_ += period.toSec();

    if (time_ == 0.0)
    {
      q_start_ = Vector7d(robot_state.q.data());
      delta_q_ = q_goal_ - q_start_;
      calculate_synchronized_values();
    }

    Vector7d delta_q_d;
    bool motion_finished = calculate_desired_values(time_, &delta_q_d);

    std::array<double, 7> joint_positions;
    Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
    franka::JointPositions output(joint_positions);
    output.motion_finished = motion_finished;
    return output;
  }

};  // namespace sdu_controllers::hal
