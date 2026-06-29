"""
Impedance controller hardware example — UR5e following a Cartesian circle.

The robot tracks a circular reference trajectory in the XY plane while the
impedance controller enforces a desired mechanical impedance between the
end-effector and the environment (here: free-space, so h_d_e = 0).
"""

import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_control import RTDEControlInterface as RTDEControl
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

def clamp_array(value, clamp_value):
    pdiff = value - clamp_value
    mdiff = value + clamp_value
    ret = value.copy()
    j = 0
    while j < len(value):
        if pdiff[j] > 0:
            ret[j] = clamp_value[j]
        elif mdiff[j] < 0:
            ret[j] = -clamp_value[j]
        
        j = j + 1
    
    return ret

# ---------------------------------------------------------------------------
# Main simulation
# ---------------------------------------------------------------------------

def main():
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"

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

    rtde_receive = RTDEReceive(robot_ip)
    rtde_control = RTDEControl(robot_ip)
    time.sleep(0.2)
    rtde_control.zeroFtSensor()
    time.sleep(0.2)

    # Robot model (UR5e)
    robot_model = sdu_controllers.models.URRobotModel(sdu_controllers.models.RobotType.ur5e)
    DOF = robot_model.get_dof()

    # UR5e max and min torque see https://www.universal-robots.com/articles/ur/robot-care-maintenance/max-joint-torques-cb3-and-e-series/
    u_max = np.array([150.0, 150.0, 150.0, 28.0, 28.0, 28.0])

    # Controller gains
    Kp_pos, Kp_orient = 1000.0, 15.0

    Kp = np.zeros((6, 6))
    Kp[0:3, 0:3] = np.eye(3) * Kp_pos
    Kp[3:6, 3:6] = np.eye(3) * Kp_orient

    # Desired inertia: 2.5 kg for translational axes, 0.5 kg·m2 for rotational
    Md_pos_val, Md_orient_val = 2.5, 0.5
    Md = np.zeros((6, 6))
    Md[0:3, 0:3] = np.eye(3) * Md_pos_val
    Md[3:6, 3:6] = np.eye(3) * Md_orient_val

    # Critical damping: D = 2 * sqrt(M * K)
    Kd_pos    = 2.0 * np.sqrt(Md_pos_val    * Kp_pos)
    Kd_orient = 2.0 * np.sqrt(Md_orient_val * Kp_orient)

    Kd = np.zeros((6, 6))
    Kd[0:3, 0:3] = np.eye(3) * Kd_pos
    Kd[3:6, 3:6] = np.eye(3) * Kd_orient

    controller = sdu_controllers.controllers.ImpedanceController(
        Kp, Kd, Md, robot_model,
        sdu_controllers.controllers.OrientationRepresentation.QUATERNION)

    # External wrench on the end-effector (zero - free-space motion)
    he    = np.zeros(6)
    h_d_e = np.zeros(6)   # desired contact wrench (zero - pure motion tracking)

    q_init = [-np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, 0.0]

    # Move robot to init position
    rtde_control.moveJ(q_init, 0.5, 0.5)

    # Circle centre from initial FK position.
    # Shift by -radius in x so the robot starts exactly on the circle at t=0.
    actual_tcp_pose = rtde_receive.getActualTCPPose()
    center = actual_tcp_pose[0:3]
    center[0] -= radius

    # Desired orientation: constant at the initial FK orientation (as quaternion).
    # Convention [w, x, y, z] — matches AdmittanceControllerPosition and step().
    quat_d = R.from_rotvec(actual_tcp_pose[3:6]).as_quat(scalar_first=True)

    actual_traj  = []
    desired_traj = []
    time_log     = []
    tau_log      = []

    # -----------------------------------------------------------------------
    # Control loop
    # -----------------------------------------------------------------------
    try:
        for step in range(steps):
            start_time = rtde_control.initPeriod()
            t = step * dt

            # Desired pose, velocity and acceleration
            pos_d, dpos_d, ddpos_d = circle_trajectory(center, radius, omega, t, ramp_time)

            x_d   = np.concatenate([pos_d,   np.zeros(3)])  # orientation part unused in QUATERNION mode
            dx_d  = np.concatenate([dpos_d,  np.zeros(3)])
            ddx_d = np.concatenate([ddpos_d, np.zeros(3)])

            # Measured state
            q_meas = rtde_receive.getActualQ()
            dq_meas = rtde_receive.getActualQd()

            # Impedance controller step (QUATERNION mode)
            controller.step(x_d, dx_d, ddx_d, q_meas, dq_meas, h_d_e, quat_d)
            y = controller.get_output()          # joint accelerations

            # Inverse dynamics --> joint torques
            tau = robot_model.inverse_dynamics(q_meas, dq_meas, y, he)

            # Clamp torques
            tau_clamped = clamp_array(tau, u_max)

            rtde_control.directTorque(tau_clamped.tolist())

            # Log data
            pos_actual = np.array(rtde_receive.getActualTCPPose()[0:3])
            actual_traj.append(pos_actual.copy())
            desired_traj.append(pos_d.copy())
            tau_log.append(tau.copy())
            time_log.append(t)

            # wait control cycle
            rtde_control.waitPeriod(start_time)

    except KeyboardInterrupt:
            rtde_control.directTorque([0.0] * DOF)
            # Perform a move to the current joint position to exit torque mode.
            rtde_control.stopJ(10)
            rtde_control.moveJ(rtde_receive.getActualQ)
    finally:
            rtde_control.directTorque([0.0] * DOF)
            # Perform a move to the current joint position to exit torque mode.
            rtde_control.stopJ(10)
            rtde_control.moveJ(rtde_receive.getActualQ)


    # -----------------------------------------------------------------------
    # Plot results
    # -----------------------------------------------------------------------
    actual_traj  = np.array(actual_traj)
    desired_traj = np.array(desired_traj)
    time_log     = np.array(time_log)
    tau_log      = np.array(tau_log)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.canvas.manager.set_window_title('Impedance Controller Circle Example') 
    # XY plane — circle tracking
    ax = axes[0]
    ax.plot(desired_traj[:, 0], desired_traj[:, 1], 'r--', label='Desired', linewidth=1.5)
    ax.plot(actual_traj[:, 0],  actual_traj[:, 1],  'b-',  label='Actual',  linewidth=1.5)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Cartesian trajectory (XY plane)')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True)

    # Position error over time
    ax = axes[1]
    error = np.linalg.norm(actual_traj - desired_traj, axis=1) * 1e3   # mm
    ax.plot(time_log, error, 'k-', linewidth=1.2)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Position error [mm]')
    ax.set_title('Cartesian position tracking error')
    ax.grid(True)

    # Joint Torques over time
    ax = axes[2]
    for i in range(DOF):
        ax.plot(time_log, tau_log[:, i], label=f'Joint {i+1}')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Torque [Nm]')
    ax.set_title('Joint Torques (tau)')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.savefig('impedance_controller_circle.png', dpi=150)
    plt.show()


if __name__ == '__main__':
    main()
