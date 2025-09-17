import numpy as np
import time
from numpy import genfromtxt
import sdu_controllers

my_data = genfromtxt('examples/data/joint_trajectory_safe.csv', delimiter=',')

freq = 500.0
dt = 1.0 / freq
Kp_val = 100.0
Ki_val = 0
Kd_val = 2 * np.sqrt(Kp_val)
N_val = 1

Kp = np.diag([Kp_val, Kp_val, Kp_val, Kp_val, Kp_val, Kp_val])
Ki = np.diag([Ki_val, Ki_val, Ki_val, Ki_val, Ki_val, Ki_val])
Kd = np.diag([Kd_val, Kd_val, Kd_val, Kd_val, Kd_val, Kd_val])
N = np.diag([N_val, N_val, N_val, N_val, N_val, N_val])

ur_robot = sdu_controllers.models.URRobotModel()
pd_controller = sdu_controllers.controllers.PIDController(Kp,Ki, Kd, N,dt)
inv_dyn_jnt_space = sdu_controllers.math.InverseDynamicsJointSpace(ur_robot)
fwd_dyn = sdu_controllers.math.ForwardDynamics(ur_robot)

q = np.array([0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0])
dq = np.zeros(6)

start_total = time.time()
for row in my_data:
    q_d = np.array(row[0:6])
    dq_d = np.array(row[6:12])
    ddq_d = np.array(row[12:18])

    u_ff = ddq_d
    pd_controller.step(q_d, dq_d, u_ff, q, dq)
    y = pd_controller.get_output()
    print('y:', y)
    tau = inv_dyn_jnt_space.inverse_dynamics(y, q, dq)
    print('tau:', tau)
    # Simulation
    ddq = fwd_dyn.forward_dynamics(q, dq, tau)
    # integrate to get velocity
    dq += ddq * dt
    # integrate to get position
    q += dq * dt

input("Press enter to quit!")
del ur_robot
del pd_controller
del inv_dyn_jnt_space

