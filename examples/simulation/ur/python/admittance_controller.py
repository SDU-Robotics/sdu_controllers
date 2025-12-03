import numpy as np
import matplotlib.pyplot as plt
import sdu_controllers

def get_circle_target(pose, time_step, radius=0.1, freq=0.5):
    x = pose[0] + radius * np.cos(2 * np.pi * freq * time_step)
    y = pose[0] + radius * np.sin(2 * np.pi * freq * time_step)

    return np.array([x, y, pose[2]])

def main():
    frequency = 500.0
    dt = 1. / frequency
    steps = 2 * frequency

    start_position = np.array([0.3, 0.3, 0.3])

    ref_traj = []
    for t in range(int(steps)):
        x_desired = get_circle_target(start_position, t*dt)
        ref_traj.append(x_desired)

    adm_controller = sdu_controllers.controllers.AdmittanceControllerCartesiann(frequency)
    adm_controller.set_mass_matrix_position(np.identity(3) * 22.5)
    adm_controller.set_stiffness_matrix_position(np.identity(3) * 54)
    adm_controller.set_damping_matrix_position(np.identity(3) * 160)

    adm_controller.set_mass_matrix_orientation(np.identity(3) * 0.25)
    adm_controller.set_stiffness_matrix_orientation(np.identity(3) * 10)
    adm_controller.set_damping_matrix_orientation(np.identity(3) * 10)

    adm_pos = get_circle_target(start_position, 0)
    quat_init = np.array([1.0, 0.0, 0.0, 0.0])

    f = np.zeros(3)
    mu = np.zeros(3)

    adm_traj = []
    for t in range(int(steps)):
        x_desired = get_circle_target(start_position, t * dt)
        if 0.29 < adm_pos[0] < 0.31 and 0.39 < adm_pos[1] < 0.41:
            f[1] = 40
        else:
            f[1] = 0

        # Step controller
        adm_controller.step(f, mu, x_desired, quat_init)
        u = adm_controller.get_output()

        adm_pos = u[0:3]

        adm_traj.append(adm_pos)

    ref_traj = np.array(ref_traj)
    adm_traj = np.array(adm_traj)

    plt.figure(figsize=(8, 8))
    plt.plot(ref_traj[:, 0], ref_traj[:, 1], label="desired trajectory", color='red')
    plt.plot(adm_traj[:, 0], adm_traj[:, 1], label="admittance trajectory", color='blue')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()