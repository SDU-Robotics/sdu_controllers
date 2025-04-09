#pragma once
#ifndef SDU_CONTROLLERS_HAL_FRANKA_ROBOT_HPP
#define SDU_CONTROLLERS_HAL_FRANKA_ROBOT_HPP

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

#include <Eigen/Dense>
#include <sdu_controllers/hal/robot.hpp>
#include <sdu_controllers/math/pose.hpp>
#include <string>
#include <vector>
#include <thread>

namespace sdu_controllers::hal
{
  /**
   * This class provides an interface for controlling a Franka Emika Robot. The interface
   * is using the FCI (Franka Control Interface) C++ library to control and receive data
   * from the robot.
   *
   * The interface provides different online control modes:
   *  - TORQUE
   *
   * where a target reference for the given control mode is set using set_joint_pos_ref(),
   * set_cartesian_pose_ref(), set_joint_vel_ref() or set_cartesian_vel_ref(). To perform the actual control,
   * the step() function of the interface must be called from the real-time thread
   * that should be controlling robot.
   *
   * Besides the online real-time control modes one can also move the robot using
   * the move_joints() or move_cartesian(), which uses the builtin robot controller
   * functions to perform the movements (typically using a trapezoidal velocity
   * interpolation).
   */

  class FrankaRobot : Robot
  {
   public:
    /**
     * A constant that defines the number of degrees of freedom (DOF) the robot have.
     */
    static const uint16_t ROBOT_DOF = 7;

    /**
     * The ControlStates enum defines the available control states for the robot control.
     */
    enum class ControlStates
    {
      NONE,
      INIT,
      STOPPED,
      RUNNING,
      ERROR
    };

    /**
     * The ControlMode enum defines the available control modes for this particular robot.
     */
    enum class ControlMode
    {
      UNDEFINED,
      JOINT_POSITION,
      JOINT_VELOCITY,
      CARTESIAN_POSE,
      CARTESIAN_VELOCITY,
      TORQUE
    };

    /**
     * FrankaRobot constructor - creates an interface for controlling the robot at
     * the specified IP-address.
     * @param ip The IP-address of the robot.
     * @param control_frequency The robot control frequency, defaults to 1000Hz.
     */
    explicit FrankaRobot(const std::string& ip, double control_frequency = 1000.0);

    /**
     * Sets a default collision behavior, joint impedance and Cartesian impedance.
     */
    void set_default_behavior();
    
    /**
     * Control function that runs in the control_thread_.
     */
    void control();

    /**
     * Callback function for the joint position control loop.
     */
    franka::Torques joint_torque_control_cb(const franka::RobotState& state, franka::Duration /*period*/);

    /**
     * Callback function for the joint position control loop.
     */
    franka::JointPositions joint_position_control_cb(const franka::RobotState& state, franka::Duration /*period*/);

    /**
     * Callback function for the joint velocity control loop.
     */
    franka::JointVelocities joint_velocity_control_cb(const franka::RobotState& state, franka::Duration /*period*/);

    /**
     * Callback function for the cartesian pose control loop.
     */
    franka::CartesianPose cartesian_pose_control_cb(const franka::RobotState& state, franka::Duration /*period*/);

    /**
     * Callback function for the cartesian velocity control loop.
     */
    franka::CartesianVelocities cartesian_velocity_control_cb(const franka::RobotState& state, franka::Duration /*period*/);

    /**
     * the step() function should be called regularly at a fixed rate by the
     * application control thread to ensure the state machine is updated and
     * that the target references is sent to the physical robot.
     */
    void step();

    /**
     * Set the robot control mode.
     *
     * Allows you to set the control mode of the given robot. See ControlMode
     * @param control_mode A control mode defined in ControlMode.
     */
    bool set_control_mode(ControlMode control_mode);

    /**
     * Start control - signals the control state machine to transition from STOPPED to the RUNNING state.
     */
    bool start_control();

    /**
     * Stop control - signals the control state machine to transition from RUNNING to the STOPPED state.
     */
    bool stop_control();

    /**
     * Set a desired joint torque reference \f$tau_{d}\f$.
     *
     * @param \f$tau_{d}\f$ specifies desired joint torques [N].
     */
    bool set_joint_torque_ref(const Eigen::Vector<double, ROBOT_DOF>& tau_d);

    /**
     * Set a joint position reference target
     *
     * @param q specifies joint positions of the robot axes [radians].
     */
    bool set_joint_pos_ref(const Eigen::Vector<double, ROBOT_DOF>& q);

    /**
     * Set a cartesian pose reference target (performs inverse kinematics on the robot
     * to produce a target in joint-space).
     *
     * @param pose specifies the target pose with position in meters and orientation as a
     * quaternion with scalar-first (WXYZ).
     */
    bool set_cartesian_pose_ref(const sdu_controllers::math::Pose& pose);

    /**
     * Set a joint velocity reference target.
     *
     * Accelerate linearly in joint space and continue with constant joint speed.
     *
     * @param dq specifies the target joint velocities \f$\dot{q}\f$.
     * @param acceleration joint acceleration [rad/s^2] (of leading axis)
     */
    bool set_joint_vel_ref(const Eigen::Vector<double, ROBOT_DOF>& dq, double acceleration = 0.5);

    /**
     * Set a cartesian velocity reference target.
     *
     * Accelerate linearly in Cartesian space and continue with constant tool speed.
     *
     * @param xd specifies the target tcp speed in [m/s].
     * @param acceleration tool position acceleration [m/s^2].
     */
    bool set_cartesian_vel_ref(const Eigen::Vector<double, ROBOT_DOF>& xd, double acceleration = 0.25);

    /**
     * Move to a given joint position in joint-space
     *
     * @param q specifies joint positions of the robot axes [radians].
     * @param speed_factor general speed factor in range [0, 1].
     */
    bool move_joints(
        const Eigen::Vector<double, ROBOT_DOF>& q,
        double speed_factor = 0.1);

    /**
     * Move to a given position in cartesian space
     *
     * @param pose specifies the target pose with position in meters and orientation as a
     * quaternion with scalar-first (WXYZ).
     * @param speed_factor general speed factor in range [0, 1].
     */
    bool
    move_cartesian(const math::Pose& pose, double velocity = 0.25, double acceleration = 1.2, bool asynchronous = false);

    /**
     * Get the joint torques
     */
    Eigen::VectorXd get_joint_torques();

    /**
     * Get the joint positions
     */
    Eigen::VectorXd get_joint_positions();

    /**
     * Get the joint velocities
     */
    Eigen::VectorXd get_joint_velocities();

    /**
     * Get the cartesian pose with position in meters and orientation as Quaterniond (see the Pose class).
     */
    math::Pose get_cartesian_tcp_pose();

    /**
     * @returns Generalized forces in the TCP.
     */
    std::vector<double> get_tcp_forces();

   private:
    franka::Robot robot_;
    franka::Model robot_model_;
    franka::RobotState robot_state_;
    double control_frequency_;
    double dt_;
    ControlMode control_mode_;
    ControlStates curr_state_;
    ControlStates prev_state_;
    bool start_control_;
    bool stop_control_;
    bool motion_finished_;
    Eigen::Vector<double, ROBOT_DOF> joint_pos_ref_;
    Eigen::Vector<double, ROBOT_DOF> joint_vel_ref_;
    math::Pose cartesian_pose_ref_;
    Eigen::Vector<double, ROBOT_DOF> cartesian_vel_ref_;
    Eigen::Vector<double, ROBOT_DOF> joint_torque_ref_;
    std::thread control_thread_;
  };

  /**
   * Copyright (c) 2023 Franka Robotics GmbH
   *
   * An example showing how to generate a joint pose motion to a goal position. Adapted from:
   * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
   * (Kogan Page Science Paper edition).
   */
  class MotionGenerator
  {
   public:
    /**
     * Creates a new MotionGenerator instance for a target q.
     *
     * @param[in] speed_factor General speed factor in range [0, 1].
     * @param[in] q_goal Target joint positions.
     */
    MotionGenerator(double speed_factor, const std::array<double, 7> q_goal);

    /**
     * Sends joint position calculations
     *
     * @param[in] robot_state Current state of the robot.
     * @param[in] period Duration of execution.
     *
     * @return Joint positions for use inside a control loop.
     */
    franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

   private:
    using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
    using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

    bool calculate_desired_values(double t, Vector7d* delta_q_d) const;
    void calculate_synchronized_values();

    static constexpr double k_delta_q_motion_finished = 1e-6;
    const Vector7d q_goal_;

    Vector7d q_start_;
    Vector7d delta_q_;

    Vector7d dq_max_sync_;
    Vector7d t_1_sync_;
    Vector7d t_2_sync_;
    Vector7d t_f_sync_;
    Vector7d q_1_;

    double time_ = 0.0;

    Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
    Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
    Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
  };

}  // namespace sdu_controllers::hal

#endif  // SDU_CONTROLLERS_HAL_FRANKA_ROBOT_HPP
