"""Analytic reference trajectory helpers shared by sdu_controllers examples & tutorials.

These are simple, analytic reference trajectories computed on the fly (no need to
pre-generate/persist to a file, unlike arbitrary offline-planned trajectories).
They mirror the C++ helpers in ``sdu_controllers/math/trajectory_generation.hpp``.
"""

import numpy as np

__all__ = ("circular_trajectory", "yaw_oscillation")


def circular_trajectory(center: np.ndarray, radius: float, omega: float, t: float, ramp_time: float):
    """Evaluate a circular Cartesian trajectory (in the XY plane) at time t.

    The angular speed ramps up from 0 to omega over ramp_time using a quintic
    profile w(t) = omega * (10u^3 - 15u^4 + 6u^5), u = t/ramp_time, which is
    C2 continuous (zero acceleration/jerk at the ramp endpoints) and therefore
    avoids a torque discontinuity at the start of the motion.

    :param center: circle centre (x0, y0, z0) in metres.
    :param radius: circle radius in metres.
    :param omega: steady-state angular velocity (rad/s).
    :param t: current time (s).
    :param ramp_time: ramp duration (s) for the angular speed to go from 0 to omega.
    :returns: (pos, vel, acc) each a 3-vector.
    """
    if ramp_time > 0.0 and t < ramp_time:
        u = np.clip(t / ramp_time, 0.0, 1.0)
        w = omega * (10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5)
        alpha = omega * (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / ramp_time
        theta = omega * ramp_time * (2.5 * u**4 - 3.0 * u**5 + u**6)
    else:
        w = omega
        alpha = 0.0
        # Keep phase continuous with the ramp profile at t = ramp_time.
        theta = omega * (t - 0.5 * ramp_time)

    c = np.cos(theta)
    s = np.sin(theta)

    pos = np.array([center[0] + radius * c, center[1] + radius * s, center[2]])
    vel = np.array([-radius * s * w, radius * c * w, 0.0])
    acc = np.array([-radius * c * w**2 - radius * s * alpha, -radius * s * w**2 + radius * c * alpha, 0.0])
    return pos, vel, acc


def yaw_oscillation(R0: np.ndarray, axis_base: np.ndarray, amplitude: float, omega: float, t: float):
    """Evaluate a small sinusoidal yaw oscillation about a fixed axis.

    Useful for exciting the orientation channel of Cartesian controllers (OSC,
    Impedance) alongside circular_trajectory(), e.g. to demonstrate the
    QUATERNION orientation representation.

    :param R0: reference orientation (3x3 rotation matrix) at yaw = 0.
    :param axis_base: rotation axis expressed in the base frame (unit vector).
    :param amplitude: yaw oscillation amplitude (rad).
    :param omega: yaw oscillation angular frequency (rad/s).
    :param t: current time (s).
    :returns: (quat_d [w, x, y, z], omega_d, domega_d).
    """
    from scipy.spatial.transform import Rotation as Rot

    yaw = amplitude * np.sin(omega * t)
    yaw_dot = amplitude * omega * np.cos(omega * t)
    yaw_ddot = -amplitude * omega * omega * np.sin(omega * t)

    omega_d = axis_base * yaw_dot
    domega_d = axis_base * yaw_ddot

    r_rel = Rot.from_rotvec(axis_base * yaw).as_matrix()
    quat_xyzw = Rot.from_matrix(r_rel @ R0).as_quat()
    quat_d = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])  # [w, x, y, z]
    return quat_d, omega_d, domega_d
