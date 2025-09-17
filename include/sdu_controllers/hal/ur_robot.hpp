#pragma once
#ifndef SDU_CONTROLLERS_HAL_UR_ROBOT_HPP
#define SDU_CONTROLLERS_HAL_UR_ROBOT_HPP

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <Eigen/Dense>
#include <chrono>
#include <sdu_controllers/hal/robot.hpp>
#include <sdu_controllers/math/pose.hpp>
#include <string>
#include <vector>

namespace sdu_controllers::hal
{
  /**
   * This class provides an interface for controlling Universal Robots. The interface
   * is using the ur_rtde library to control and receive data from the robot.
   *
   * The interface provides different online control modes:
   *
   *  - JOINT_POSITION
   *  - CARTESIAN
   *  - VELOCITY
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

  class URRobot : Robot
  {
   public:
    /**
     * A constant that defines the number of degrees of freedom (DOF) the robot have.
     */
    static const uint16_t ROBOT_DOF = 6;

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
      CARTESIAN_POSE,
      JOINT_VELOCITY,
      CARTESIAN_VELOCITY,
      TORQUE
    };

    /**
     * URRobot constructor - creates a UR RTDE Interface for the robot at the specified IP-address.
     * @param ip The IP-address of the robot.
     * @param control_frequency The robot control frequency, defaults to 500Hz.
     */
    explicit URRobot(const std::string& ip, double control_frequency = 500.0);

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
     * @brief This function is used in combination with wait_period() and is used to get the start of a control period /
     * cycle.
     */
    std::chrono::steady_clock::time_point init_period();

    /**
     * @brief Used for waiting the rest of the control period, set implicitly as dt = 1 / frequency. A combination of
     * sleeping and spinning are used to achieve the lowest possible jitter. The function is especially useful for a
     * realtime control loop. NOTE: the function is to be used in combination with the init_period().

     * @param t_cycle_start the start of the control period. Typically given as dt = 1 / frequency.
     */
    void wait_period(const std::chrono::steady_clock::time_point& t_cycle_start);

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
     * Set a joint torque reference target.
     *
     * Commands joint torques directly.
     *
     * @param dq specifies the target joint velocities \f$\dot{q}\f$.
     * @param acceleration joint acceleration [rad/s^2] (of leading axis)
     */
    bool set_joint_torque_ref(const Eigen::Vector<double, ROBOT_DOF>& torques);

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
     * @brief Zeroes the TCP force/torque measurement from the builtin force/torque sensor by subtracting the current
     * measurement from the subsequent.
     */
    void zero_ft_sensor();

    /**
     * Move to a given joint position in joint-space
     * @param q specifies joint positions of the robot axes [radians].
     * @param velocity joint velocity [rad/s]
     * @param acceleration joint acceleration [rad/s^2]
     * @param asynchronous a bool specifying if the move command should be asynchronous.
     * Default is false, this means the function will block until the movement has completed.
     */
    bool move_joints(
        const Eigen::Vector<double, ROBOT_DOF>& q,
        double velocity = 1.05,
        double acceleration = 1.4,
        bool asynchronous = false);

    /**
     * Move to a given position in cartesian space
     * @param pose specifies the target pose with position in meters and orientation as a
     * quaternion with scalar-first (WXYZ).
     * @param velocity tool velocity [m/s]
     * @param acceleration tool acceleration [m/s^2]
     * @param asynchronous a bool specifying if the move command should be asynchronous. Default is false, this means the
     * function will block until the movement has completed.
     */
    bool
    move_cartesian(const math::Pose& pose, double velocity = 0.25, double acceleration = 1.2, bool asynchronous = false);

    /**
     * Get the joint torques
     */
    Eigen::VectorXd get_joint_torques();

    Eigen::VectorXd get_actual_joint_currents();

    Eigen::VectorXd get_target_joint_currents();

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
    std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
    std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control_;
    double control_frequency_;
    double dt_;
    ControlMode control_mode_;
    ControlStates curr_state_;
    ControlStates prev_state_;
    bool start_control_;
    bool stop_control_;
    Eigen::Vector<double, ROBOT_DOF> joint_pos_ref_;
    math::Pose cartesian_pose_ref_;
    Eigen::Vector<double, ROBOT_DOF> joint_vel_ref_;
    Eigen::Vector<double, ROBOT_DOF> cartesian_vel_ref_;
    Eigen::Vector<double, ROBOT_DOF> joint_torque_ref_;

    double servo_vel_;
    double servo_acc_;
    double servo_p_gain_;
    double servo_lookahead_t_;

    double deceleration_rate_;
    double vel_tool_acceleration_;
  };

}  // namespace sdu_controllers::hal

#endif  // SDU_CONTROLLERS_HAL_UR_ROBOT_HPP
