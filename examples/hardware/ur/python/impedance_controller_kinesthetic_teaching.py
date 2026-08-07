"""
Impedance controller hardware example - UR5e kinesthetic teaching.

The robot follows no reference trajectory. Instead the impedance controller is
run with zero Cartesian stiffness so the arm offers no resistance and can be
freely moved around by hand (kinesthetic teaching). Only damping and the
measured end-effector wrench act, so the robot stays wherever it is placed.
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
# Helpers
# ---------------------------------------------------------------------------

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

    # Loop parameters
    freq      = 500.0
    dt        = 1.0 / freq
    total_t   = 100.0                          # seconds
    steps     = int(total_t * freq)

    rtde_r = RTDEReceive(robot_ip)
    rtde_c = RTDEControl(robot_ip)

    # Robot model (UR5e)
    robot_model = sdu_controllers.models.URRobotModel(sdu_controllers.models.RobotType.ur5e)
    DOF = robot_model.get_dof()

    # UR5e max and min torque
    u_max = np.array([150.0, 150.0, 150.0, 28.0, 28.0, 28.0])

    # Controller gains - kinesthetic teaching.
    # Zero stiffness so the robot offers no resistance and stays wherever it is
    # placed. Only damping remains, which keeps the motion smooth and stable.
    Kp = np.zeros((6, 6))

    # Desired inertia for translational and rotational axes.
    Md_pos_val, Md_orient_val = 10.0, 0.2
    Md = np.zeros((6, 6))
    Md[0:3, 0:3] = np.eye(3) * Md_pos_val
    Md[3:6, 3:6] = np.eye(3) * Md_orient_val

    # Damping (set directly since there is no stiffness to critically damp).
    Kd_pos, Kd_orient = 80.0, 3.0
    Kd = np.zeros((6, 6))
    Kd[0:3, 0:3] = np.eye(3) * Kd_pos
    Kd[3:6, 3:6] = np.eye(3) * Kd_orient

    controller = sdu_controllers.controllers.ImpedanceController(
        Kp, Kd, Md, robot_model,
        sdu_controllers.controllers.OrientationRepresentation.QUATERNION)

    # External wrench on the end-effector (measured, filtered in the control loop)
    he    = np.zeros(6)
    h_d_e = np.zeros(6)   # desired contact wrench

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

    # Fixed reference pose taken from the initial robot pose
    actual_tcp_pose = rtde_r.getActualTCPPose()
    x_d = np.concatenate([np.array(actual_tcp_pose[0:3]), np.zeros(3)])

    # Desired orientation: constant at the initial FK orientation (as quaternion).
    # Convention [w, x, y, z] - matches AdmittanceControllerPosition and step().
    quat_d = R.from_rotvec(actual_tcp_pose[3:6]).as_quat(scalar_first=True)

    # No desired motion - hold zero reference velocity and acceleration.
    dx_d  = np.zeros(6)
    ddx_d = np.zeros(6)

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
