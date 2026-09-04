#pragma once
#ifndef SDU_CONTROLLERS_MATH_TRAJECTORY_GENERATION_HPP
#define SDU_CONTROLLERS_MATH_TRAJECTORY_GENERATION_HPP

#include <Eigen/Dense>
#include <cmath>

namespace sdu_controllers::math
{
  /**
   * @brief Evaluate a circular Cartesian trajectory (in the XY plane) at time t.
   *
   * The angular speed ramps up from 0 to omega over ramp_time using a quintic
   * profile w(t) = omega * (10u^3 - 15u^4 + 6u^5), u = t/ramp_time, which is
   * C2 continuous (zero acceleration/jerk at the ramp endpoints) and therefore
   * avoids a torque discontinuity at the start of the motion.
   *
   * This is computed analytically at every call - there is no need to
   * pre-generate or persist this trajectory to a file, unlike arbitrary
   * offline-planned joint trajectories.
   *
   * @param center     Circle centre (x0, y0, z0) in metres.
   * @param radius     Circle radius in metres.
   * @param omega      Steady-state angular velocity (rad/s).
   * @param t          Current time (s).
   * @param ramp_time  Ramp duration (s) for the angular speed to go from 0 to omega.
   * @param pos        Output: desired position (3-vector).
   * @param vel        Output: desired velocity (3-vector).
   * @param acc        Output: desired acceleration (3-vector).
   */
  inline void circular_trajectory(
      const Eigen::Vector3d &center,
      double radius,
      double omega,
      double t,
      double ramp_time,
      Eigen::Vector3d &pos,
      Eigen::Vector3d &vel,
      Eigen::Vector3d &acc)
  {
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
      // Keep phase continuous with the ramp profile at t = ramp_time.
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

  /**
   * @brief Evaluate a small sinusoidal yaw oscillation about a fixed axis, expressed
   * as a quaternion, angular velocity and angular acceleration.
   *
   * Useful for exciting the orientation channel of Cartesian controllers (OSC,
   * Impedance) alongside a positional circular_trajectory(), e.g. to demonstrate
   * the QUATERNION orientation representation.
   *
   * @param R0            Reference orientation (3x3 rotation matrix) at yaw = 0.
   * @param axis_base     Rotation axis expressed in the base frame (unit vector).
   * @param amplitude     Yaw oscillation amplitude (rad).
   * @param omega         Yaw oscillation angular frequency (rad/s).
   * @param t             Current time (s).
   * @param quat_d        Output: desired orientation quaternion [w, x, y, z].
   * @param omega_d       Output: desired angular velocity (3-vector, base frame).
   * @param domega_d      Output: desired angular acceleration (3-vector, base frame).
   */
  inline void yaw_oscillation(
      const Eigen::Matrix3d &R0,
      const Eigen::Vector3d &axis_base,
      double amplitude,
      double omega,
      double t,
      Eigen::Vector4d &quat_d,
      Eigen::Vector3d &omega_d,
      Eigen::Vector3d &domega_d)
  {
    const double yaw = amplitude * std::sin(omega * t);
    const double yaw_dot = amplitude * omega * std::cos(omega * t);
    const double yaw_ddot = -amplitude * omega * omega * std::sin(omega * t);

    omega_d = axis_base * yaw_dot;
    domega_d = axis_base * yaw_ddot;

    Eigen::Matrix3d R_rel = Eigen::AngleAxisd(yaw, axis_base).toRotationMatrix();
    Eigen::Quaterniond q(R_rel * R0);
    q.normalize();
    quat_d << q.w(), q.x(), q.y(), q.z();
  }

}  // namespace sdu_controllers::math

#endif  // SDU_CONTROLLERS_MATH_TRAJECTORY_GENERATION_HPP
