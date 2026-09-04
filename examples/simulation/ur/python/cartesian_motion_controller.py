"""Operational-space controller simulation example - UR5e following a Cartesian circle.

The robot tracks a circular reference trajectory in the XY plane using the
operational-space (inverse-dynamics) controller in ZYZ orientation mode.
"""

import numpy as np
import sdu_controllers
from sdu_controllers.common import circular_trajectory

PI = 3.14159265358979323846


def rotation_to_zyz(rot: np.ndarray) -> np.ndarray:
    """Extract ZYZ Euler angles from a 3x3 rotation matrix (matches Eigen convention)."""
    theta = np.arccos(np.clip(rot[2, 2], -1.0, 1.0))
    if abs(np.sin(theta)) < 1e-10:
        phi = 0.0
        psi = np.arctan2(-rot[0, 1], rot[0, 0]) if rot[2, 2] > 0 else np.arctan2(rot[0, 1], -rot[0, 0])
    else:
        phi = np.arctan2(rot[2, 1], rot[2, 0])
        psi = np.arctan2(rot[1, 2], -rot[0, 2])
    return np.array([phi, theta, psi])


def main():
    freq = 500.0
    dt = 1.0 / freq
    total_t = 4.0  # seconds (two full circles)
    steps = int(total_t * freq)

    # Circle parameters
    radius = 0.05
    circle_freq = 0.5  # Hz
    omega = 2.0 * PI * circle_freq
    ramp_time = 0.3

    robot_model = sdu_controllers.models.URRobotModel(sdu_controllers.models.RobotType.ur5e)
    dof = robot_model.get_dof()

    Kp_pos, Kp_orient = 16250.0, 16250.0
    Kd_pos, Kd_orient = 200.0, 3.0

    Kp = np.eye(6)
    Kp[0:3, 0:3] = np.eye(3) * Kp_pos
    Kp[3:6, 3:6] = np.eye(3) * Kp_orient
    Kd = np.eye(6)
    Kd[0:3, 0:3] = np.eye(3) * Kd_pos
    Kd[3:6, 3:6] = np.eye(3) * Kd_orient

    # ZYZ mode (default): x_d is [position(3), ZYZ angles(3)].
    controller = sdu_controllers.controllers.OperationalSpaceController(Kp, Kd, robot_model)

    q = np.array([0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0])
    dq = np.zeros(dof)
    he = np.zeros(6)

    T0 = robot_model.get_fk_solver().forward_kinematics(q)
    center = T0[:3, 3].copy()
    center[0] -= radius
    rot_zyz_d = rotation_to_zyz(T0[:3, :3])

    for step in range(steps):
        t = step * dt
        pos_d, dpos_d, ddpos_d = circular_trajectory(center, radius, omega, t, ramp_time)

        x_d = np.concatenate([pos_d, rot_zyz_d])
        dx_d = np.concatenate([dpos_d, np.zeros(3)])
        ddx_d = np.concatenate([ddpos_d, np.zeros(3)])

        controller.step(x_d, dx_d, ddx_d, q, dq)
        y = controller.get_output()

        tau = robot_model.inverse_dynamics(q, dq, y, he)
        ddq = robot_model.forward_dynamics(q, dq, tau)
        dq += ddq * dt
        q += dq * dt

    print("Simulation complete.")


if __name__ == "__main__":
    main()
