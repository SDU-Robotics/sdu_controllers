"""
Impedance controller simulation example - UR5e following a Cartesian circle.

The robot tracks a circular reference trajectory in the XY plane while the
impedance controller enforces a desired mechanical impedance between the
end-effector and the environment (here: free-space, so h_d_e = 0).
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import sdu_controllers


# ---------------------------------------------------------------------------
# Trajectory helpers
# ---------------------------------------------------------------------------

def circle_trajectory(center: np.ndarray, radius: float, omega: float, t: float, ramp_time: float):
    """Return desired Cartesian position, velocity and acceleration on a circle
    in the XY plane centred at *center* at time *t*."""
    # Quintic angular speed ramp: w(t) = omega * (10u^3 - 15u^4 + 6u^5), u=t/ramp_time.
    # This is C2 continuous (zero accel/jerk at ramp start/end), reducing torque overshoot.
    if ramp_time > 0.0 and t < ramp_time:
        u = np.clip(t / ramp_time, 0.0, 1.0)
        w = omega * (10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5)
        alpha = omega * (30.0 * u ** 2 - 60.0 * u ** 3 + 30.0 * u ** 4) / ramp_time
        theta = omega * ramp_time * (2.5 * u ** 4 - 3.0 * u ** 5 + u ** 6)
    else:
        w = omega
        alpha = 0.0
        # Keep phase continuous with ramp profile at t=ramp_time.
        theta = omega * (t - 0.5 * ramp_time)

    c = np.cos(theta)
    s = np.sin(theta)

    pos = np.array([
        center[0] + radius * c,
        center[1] + radius * s,
        center[2],
    ])
    vel = np.array([
        -radius * s * w,
         radius * c * w,
         0.0,
    ])
    acc = np.array([
        -radius * c * w ** 2 - radius * s * alpha,
        -radius * s * w ** 2 + radius * c * alpha,
         0.0,
    ])
    return pos, vel, acc


def rotation_to_zyz(R: np.ndarray) -> np.ndarray:
    """Extract ZYZ Euler angles from a 3x3 rotation matrix (matches Eigen convention)."""
    # Eigen eulerAngles(2,1,2) → phi, theta, psi  (ZYZ)
    theta = np.arccos(np.clip(R[2, 2], -1.0, 1.0))
    if abs(np.sin(theta)) < 1e-10:
        phi = 0.0
        psi = np.arctan2(-R[0, 1], R[0, 0]) if R[2, 2] > 0 else np.arctan2(R[0, 1], -R[0, 0])
    else:
        phi = np.arctan2(R[2, 1], R[2, 0])     # atan2( R[2,1],  R[2,0] )
        psi = np.arctan2(R[1, 2], -R[0, 2])    # atan2( R[1,2], -R[0,2] )
    return np.array([phi, theta, psi])


# ---------------------------------------------------------------------------
# Main simulation
# ---------------------------------------------------------------------------

def main():
    # Simulation parameters
    freq      = 500.0
    dt        = 1.0 / freq
    total_t   = 4.0                          # seconds (two full circles)
    steps     = int(total_t * freq)

    # Circle parameters
    radius       = 0.05                      # 5 cm
    circle_freq  = 0.5                       # Hz
    omega        = 2.0 * np.pi * circle_freq
    ramp_time    = 0.3                       # s

    # Orientation excitation: small sinusoid around base Z-axis.
    orient_amp_deg  = 5.0                    # deg
    orient_amp_rad  = np.deg2rad(orient_amp_deg)
    orient_freq_hz  = 0.5                    # Hz
    orient_omega    = 2.0 * np.pi * orient_freq_hz

    # Robot model (UR5e)
    robot_model = sdu_controllers.models.URRobotModel(sdu_controllers.models.RobotType.ur5e)
    DOF = robot_model.get_dof()

    # Controller gains
    Kp_pos, Kp_orient = 1000.0, 200.0

    Kp = np.zeros((6, 6))
    Kp[0:3, 0:3] = np.eye(3) * Kp_pos
    Kp[3:6, 3:6] = np.eye(3) * Kp_orient

    # Desired inertia for translational rotational axes
    Md_pos_val, Md_orient_val = 2.5, 0.5
    Md = np.zeros((6, 6))
    Md[0:3, 0:3] = np.eye(3) * Md_pos_val
    Md[3:6, 3:6] = np.eye(3) * Md_orient_val

    # Critical damping: D = 2 * sqrt(M * K)
    Kd_pos    = 2.0 * np.sqrt(Md_pos_val * Kp_pos)
    Kd_orient = 2.0 * np.sqrt(Md_orient_val * Kp_orient)

    Kd = np.zeros((6, 6))
    Kd[0:3, 0:3] = np.eye(3) * Kd_pos
    Kd[3:6, 3:6] = np.eye(3) * Kd_orient

    controller = sdu_controllers.controllers.ImpedanceController(
        Kp, Kd, Md, robot_model,
        sdu_controllers.controllers.OrientationRepresentation.QUATERNION)

    # Initial joint state 
    q  = np.array([0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0])
    dq = np.zeros(DOF)

    # External wrench on the end-effector (zero - free-space motion)
    he    = np.zeros(6)
    h_d_e = np.zeros(6)   # desired contact wrench (zero - pure motion tracking)

    # Circle centre from initial FK position.
    # Shift by -radius in x so the robot starts exactly on the circle at t=0.
    T0 = robot_model.get_fk_solver().forward_kinematics(q)
    center = T0[:3, 3].copy()
    center[0] -= radius

    # Desired orientation: constant at the initial FK orientation (as quaternion).
    # Convention [w, x, y, z] — matches AdmittanceControllerPosition and step().
    R0 = T0[:3, :3].copy()
    quat_d = R.from_matrix(R0).as_quat() # returns [x, y, z, w]
    quat_d = np.array([quat_d[3], quat_d[0], quat_d[1], quat_d[2]]) # reorder to [w, x, y, z]

    # For R_d = R0 * Rz(yaw), desired angular velocity/acceleration in base frame
    # are along the initial tool z-axis expressed in base coordinates.
    orient_axis_base = R0[:, 2].copy()

    actual_traj  = []
    desired_traj = []
    desired_eul_log = []
    actual_eul_log = []
    orient_err_eul_log = []
    orient_err_angle_log = []
    time_log     = []
    tau_log      = []

    # -----------------------------------------------------------------------
    # Control loop
    # -----------------------------------------------------------------------
    for step in range(steps):
        t = step * dt

        # Desired pose, velocity and acceleration
        pos_d, dpos_d, ddpos_d = circle_trajectory(center, radius, omega, t, ramp_time)

        # Small sinusoidal orientation command in yaw.
        yaw_d = orient_amp_rad * np.sin(orient_omega * t)
        yaw_d_dot = orient_amp_rad * orient_omega * np.cos(orient_omega * t)
        yaw_d_ddot = -orient_amp_rad * orient_omega * orient_omega * np.sin(orient_omega * t)

        omega_d = orient_axis_base * yaw_d_dot
        domega_d = orient_axis_base * yaw_d_ddot

        R_rel_d = R.from_euler('z', yaw_d).as_matrix()
        R_d = R0 @ R_rel_d
        quat_d_xyzw = R.from_matrix(R_d).as_quat()
        quat_d = np.array([quat_d_xyzw[3], quat_d_xyzw[0], quat_d_xyzw[1], quat_d_xyzw[2]])  # [w, x, y, z]

        x_d   = np.concatenate([pos_d, np.zeros(3)])  # orientation state commanded through quat_d
        dx_d  = np.concatenate([dpos_d, omega_d])
        ddx_d = np.concatenate([ddpos_d, domega_d])

        # Impedance controller step (QUATERNION mode)
        controller.step(x_d, dx_d, ddx_d, q, dq, h_d_e, quat_d)
        y = controller.get_output()          # joint accelerations

        # Inverse dynamics -> joint torques
        tau = robot_model.inverse_dynamics(q, dq, y, he)

        # Forward dynamics -> joint accelerations
        ddq = robot_model.forward_dynamics(q, dq, tau)

        # Euler integration
        dq += ddq * dt
        q  += dq * dt

        # Log actual end-effector position
        T_actual   = robot_model.get_fk_solver().forward_kinematics(q)
        pos_actual = T_actual[:3, 3]
        R_actual = T_actual[:3, :3]

        # Express orientation tracking in the same frame: relative to the initial pose R0.
        R_rel_actual = R0.T @ R_actual
        eul_rel_d = R.from_matrix(R_rel_d).as_euler('zyx')
        eul_rel_actual = R.from_matrix(R_rel_actual).as_euler('zyx')

        # Orientation error from desired to actual, in relative frame.
        R_err = R_rel_d.T @ R_rel_actual
        eul_err = R.from_matrix(R_err).as_euler('zyx')
        ang_err = np.linalg.norm(R.from_matrix(R_err).as_rotvec())

        actual_traj.append(pos_actual.copy())
        desired_traj.append(pos_d.copy())
        desired_eul_log.append(eul_rel_d)
        actual_eul_log.append(eul_rel_actual)
        orient_err_eul_log.append(eul_err)
        orient_err_angle_log.append(ang_err)
        tau_log.append(tau.copy())
        time_log.append(t)

    # -----------------------------------------------------------------------
    # Plot results
    # -----------------------------------------------------------------------
    actual_traj  = np.array(actual_traj)
    desired_traj = np.array(desired_traj)
    desired_eul_log = np.array(desired_eul_log)
    actual_eul_log = np.array(actual_eul_log)
    orient_err_eul_log = np.array(orient_err_eul_log)
    orient_err_angle_log = np.array(orient_err_angle_log)
    time_log     = np.array(time_log)
    tau_log      = np.array(tau_log)

    # Unwrap angle signals before converting to degrees for clean plots.
    desired_eul_deg = np.rad2deg(np.unwrap(desired_eul_log, axis=0))
    actual_eul_deg = np.rad2deg(np.unwrap(actual_eul_log, axis=0))
    orient_err_eul_deg = np.rad2deg(np.unwrap(orient_err_eul_log, axis=0))
    orient_err_angle_deg = np.rad2deg(orient_err_angle_log)

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.canvas.manager.set_window_title('Impedance Controller Circle Example') 
    # XY plane - circle tracking
    ax = axes[0, 0]
    ax.plot(desired_traj[:, 0], desired_traj[:, 1], 'r--', label='Desired', linewidth=1.5)
    ax.plot(actual_traj[:, 0],  actual_traj[:, 1],  'b-',  label='Actual',  linewidth=1.5)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Cartesian trajectory (XY plane)')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True)

    # Position error over time
    ax = axes[0, 1]
    error = np.linalg.norm(actual_traj - desired_traj, axis=1) * 1e3   # mm
    ax.plot(time_log, error, 'k-', linewidth=1.2)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Position error [mm]')
    ax.set_title('Cartesian position tracking error')
    ax.grid(True)

    # Joint Torques over time
    ax = axes[0, 2]
    for i in range(DOF):
        ax.plot(time_log, tau_log[:, i], label=f'Joint {i+1}')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Torque [Nm]')
    ax.set_title('Joint Torques (tau)')
    ax.legend()
    ax.grid(True)

    # Desired vs actual orientation in relative ZYX Euler angles.
    ax = axes[1, 0]
    labels = ['Yaw (z)', 'Pitch (y)', 'Roll (x)']
    colors = ['tab:red', 'tab:green', 'tab:blue']
    for i in range(3):
        ax.plot(time_log, desired_eul_deg[:, i], linestyle='--', color=colors[i], linewidth=1.3, label=f'Desired {labels[i]}')
        ax.plot(time_log, actual_eul_deg[:, i], linestyle='-', color=colors[i], linewidth=1.3, label=f'Actual {labels[i]}')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angle [deg]')
    ax.set_title('Orientation tracking (relative ZYX)')
    ax.legend()
    ax.grid(True)

    # Orientation error components in degrees.
    ax = axes[1, 1]
    for i in range(3):
        ax.plot(time_log, orient_err_eul_deg[:, i], color=colors[i], linewidth=1.4, label=f'Error {labels[i]}')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Error [deg]')
    ax.set_title('Orientation error (Euler components)')
    ax.legend()
    ax.grid(True)

    # Orientation error magnitude (geodesic angle).
    ax = axes[1, 2]
    ax.plot(time_log, orient_err_angle_deg, 'k-', linewidth=1.4, label='|R_err|')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Error [deg]')
    ax.set_title('Orientation error magnitude')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.savefig('impedance_controller_circle.png', dpi=150)
    plt.show()


if __name__ == '__main__':
    main()
