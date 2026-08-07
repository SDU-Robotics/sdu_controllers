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

def apply_deadband(wrench, force_threshold, torque_threshold):
    """Apply a symmetric deadband to a wrench: any component with magnitude below
    the corresponding threshold is set to zero. Components above the threshold are
    passed through unchanged (not offset-shifted), so contact forces are preserved.

    The first three components are treated as forces (N) and the last three as
    torques (Nm), each with its own threshold."""
    ret = wrench.copy()
    thresholds = np.array([force_threshold] * 3 + [torque_threshold] * 3)
    ret[np.abs(wrench) < thresholds] = 0.0
    return ret

# ---------------------------------------------------------------------------
# Main simulation
# ---------------------------------------------------------------------------

def main():
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"

    # Simulation parameters
    freq      = 500.0
    dt        = 1.0 / freq
    total_t   = 100.0                          # seconds
    steps     = int(total_t * freq)

    # Circle parameters
    radius       = 0.05                      # 5 cm
    circle_freq  = 0.5                       # Hz
    omega        = 2.0 * np.pi * circle_freq
    ramp_time    = 0.3                       # s

    rtde_r = RTDEReceive(robot_ip)
    rtde_c = RTDEControl(robot_ip)

    # Robot model (UR5e)
    robot_model = sdu_controllers.models.URRobotModel(sdu_controllers.models.RobotType.ur5e)
    DOF = robot_model.get_dof()

    # UR5e max and min torque
    u_max = np.array([150.0, 150.0, 150.0, 28.0, 28.0, 28.0])

    # Controller gains
    Kp_pos, Kp_orient = 1000.0, 50.0

    Kp = np.zeros((6, 6))
    Kp[0:3, 0:3] = np.eye(3) * Kp_pos
    Kp[3:6, 3:6] = np.eye(3) * Kp_orient

    # Desired inertia for translational and rotational axes.
    Md_pos_val, Md_orient_val = 10.0, 0.2
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

    # External wrench on the end-effector (measured, filtered in the control loop)
    he    = np.zeros(6)
    h_d_e = np.zeros(6)   # desired contact wrench (zero - pure motion tracking)

    # --- End-effector wrench filtering ---
    # First-order low-pass filter to suppress force/torque sensor noise.
    #   he_filt = a * he_raw + (1 - a) * he_filt_prev,   a = 1 - exp(-2*pi*fc*dt)
    ft_cutoff_freq = 10.0                    # Hz
    ft_lpf_alpha   = 1.0 - np.exp(-2.0 * np.pi * ft_cutoff_freq * dt)
    he_filtered    = np.zeros(6)
    # Deadband half-widths: ignore forces below ft_force_deadband and torques below ft_torque_deadband.
    ft_force_deadband  = 1.5                 # N
    ft_torque_deadband = 1.0                 # Nm

    q_init = [0.0, -np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, 0.0]

    # Move robot to init position
    rtde_c.moveJ(q_init, 0.5, 0.5)

    # Circle centre from initial FK position.
    # Shift by -radius in x so the robot starts exactly on the circle at t=0.
    actual_tcp_pose = rtde_r.getActualTCPPose()
    center = actual_tcp_pose[0:3]
    center[0] -= radius

    # Desired orientation: constant at the initial FK orientation (as quaternion).
    # Convention [w, x, y, z] - matches AdmittanceControllerPosition and step().
    quat_d = R.from_rotvec(actual_tcp_pose[3:6]).as_quat(scalar_first=True)

    # Zero the force-torque sensor before starting the control loop
    time.sleep(0.2)
    rtde_c.zeroFtSensor()
    time.sleep(0.2)

    # Start logging of robot data
    rtde_r.startFileRecording('robot_data.csv')

    # -----------------------------------------------------------------------
    # Control loop
    # -----------------------------------------------------------------------
    try:
        for step in range(steps):
            start_time = rtde_c.initPeriod()
            t = step * dt

            # Desired pose, velocity and acceleration
            pos_d, dpos_d, ddpos_d = circle_trajectory(center, radius, omega, t, ramp_time)

            x_d   = np.concatenate([pos_d,   np.zeros(3)])  # orientation part unused in QUATERNION mode
            dx_d  = np.concatenate([dpos_d,  np.zeros(3)])
            ddx_d = np.concatenate([ddpos_d, np.zeros(3)])

            # Measured state
            q_meas = rtde_r.getActualQ()
            dq_meas = rtde_r.getActualQd()

            # Measured external wrench at the end-effector
            he_raw = np.array(rtde_r.getActualTCPForce())

            # Low-pass filter to attenuate sensor noise.
            he_filtered = ft_lpf_alpha * he_raw + (1.0 - ft_lpf_alpha) * he_filtered

            # Deadband to reject small residual forces (N) / torques (Nm).
            he = apply_deadband(he_filtered, ft_force_deadband, ft_torque_deadband)

            # Impedance controller step (QUATERNION mode)
            controller.step(x_d, dx_d, ddx_d, q_meas, dq_meas, h_d_e, quat_d)
            y = controller.get_output()          # joint accelerations

            # Inverse dynamics --> joint torques.
            # Gravity is subtracted to avoid double-counting it in the directTorque command.
            tau = robot_model.inverse_dynamics(q_meas, dq_meas, y, he)
            tau_g = np.asarray(robot_model.get_gravity(q_meas)).reshape(-1)
            tau -= tau_g

            # Clamp torques
            tau_clamped = clamp_array(tau, u_max)

            rtde_c.directTorque(tau_clamped.tolist())

            # wait control cycle
            rtde_c.waitPeriod(start_time)

    except KeyboardInterrupt:
            print("Interrupted. Stopping...")
    finally:
            rtde_r.stopFileRecording()
            rtde_c.directTorque([0.0] * DOF)
            # Perform a move to the current joint position to exit torque mode.
            rtde_c.stopJ(10)
            rtde_c.moveJ(rtde_r.getActualQ)


if __name__ == '__main__':
    main()
