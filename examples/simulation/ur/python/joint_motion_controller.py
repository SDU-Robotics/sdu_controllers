"""Joint-space controller simulation example - UR5e tracking a sinusoidal joint trajectory.

The robot tracks q_d(t) = q0 + amplitude * sin(omega * t), applied identically to every joint.
"""

import numpy as np
import sdu_controllers

PI = 3.14159265358979323846


def main():
    freq = 500.0
    dt = 1.0 / freq
    total_t = 10.0  # seconds
    steps = int(total_t * freq)

    # Sinusoidal joint trajectory parameters
    amplitude = 0.2  # rad
    traj_freq = 0.1  # Hz
    omega = 2.0 * PI * traj_freq

    Kp_val = 1000.0
    Ki_val = 100.0
    Kd_val = 2 * np.sqrt(Kp_val)
    N_val = 1

    robot_model = sdu_controllers.models.URRobotModel(sdu_controllers.models.RobotType.ur5e)
    dof = robot_model.get_dof()

    Kp = np.eye(dof) * Kp_val
    Ki = np.eye(dof) * Ki_val
    Kd = np.eye(dof) * Kd_val
    N = np.eye(dof) * N_val

    # UR5e max torque, see https://www.universal-robots.com/articles/ur/robot-care-maintenance/max-joint-torques-cb3-and-e-series/
    u_max = np.array([150.0, 150.0, 150.0, 28.0, 28.0, 28.0])
    u_min = -u_max

    pid_controller = sdu_controllers.controllers.PIDController(Kp, Ki, Kd, N, dt, u_min, u_max)

    q0 = np.array([0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0])
    q = q0.copy()
    dq = np.zeros(dof)
    he = np.zeros(6)

    for step in range(steps):
        t = step * dt

        # Desired
        q_d = q0 + amplitude * np.sin(omega * t)
        dq_d = np.full(dof, amplitude * omega * np.cos(omega * t))
        ddq_d = np.full(dof, -amplitude * omega * omega * np.sin(omega * t))

        # Controller
        u_ff = ddq_d  # acceleration as feedforward.
        # u_ff = robot_model.get_gravity(q)  # feedforward with gravity compensation.
        pid_controller.step(q_d, dq_d, u_ff, q, dq)
        y = pid_controller.get_output()
        tau = robot_model.inverse_dynamics(q, dq, y, he)

        # Simulation of the resulting motion, used here to validate the tracking performance.
        ddq = robot_model.forward_dynamics(q, dq, tau)
        dq += ddq * dt
        q += dq * dt

    print("Simulation complete.")


if __name__ == "__main__":
    main()

