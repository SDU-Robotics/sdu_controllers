import numpy as np
import time
import csv
from numpy import genfromtxt
from scipy.spatial.transform import Rotation
import sdu_controllers


my_data = genfromtxt('examples/data/cartesian_trajectory_safe.csv', delimiter=',')

freq = 500.0
dt = 1.0 / freq

Kp_pos_val = 500
Kp_orient_val = 100
Kd_pos_val = 2 * np.sqrt(Kp_pos_val)
Kd_orient_val = 2 * np.sqrt(Kp_orient_val)
N_val = 1

Kp = np.diag([Kp_pos_val, Kp_pos_val, Kp_pos_val, Kp_orient_val, Kp_orient_val, Kp_orient_val])
Kd = np.diag([Kd_pos_val, Kd_pos_val, Kd_pos_val, Kd_orient_val, Kd_orient_val, Kd_orient_val])
# N = 

ur_robot = sdu_controllers.URRobotModel()
osc_controller = sdu_controllers.OperationalSpaceController(Kp, Kd, ur_robot)
inv_dyn_jnt_space = sdu_controllers.InverseDynamicsJointSpace(ur_robot)
fwd_dyn = sdu_controllers.ForwardDynamics(ur_robot)

q = np.array([0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0])
dq = np.zeros(6)

output_data = []
start_total = time.time()
for row in my_data:
    x_d = np.array(row[0:6])
    dx_d = np.array(row[6:12])
    ddx_d = np.array(row[12:18])

    q_meas = q
    dq_meas = dq
    # 

    # Controller 
    osc_controller.step(x_d, dx_d, ddx_d, q_meas, dq_meas)
    y = osc_controller.get_output()
    print('y:', y)
    tau = inv_dyn_jnt_space.inverse_dynamics(y, q_meas, dq_meas)
    print('tau', tau)

    # Simulation
    ddq = fwd_dyn.forward_dynamics(q, dq, tau)
    # integrate to get velocity
    dq += ddq * dt
    # integrate to get position
    q += dq * dt

    print('q:', q)
    T = sdu_controllers.forward_kinematics(q, ur_robot)
    pos = T[0:3, 3]
    print('pos:', pos)
    rot_mat = T[0:3, 0:3]
    rpy_zyz = Rotation.from_matrix(rot_mat).as_euler('zyz', degrees=False)
    rpy_zyz = rpy_zyz[[2, 1, 0]]
    print('rpy_zyz:', rpy_zyz)

    output_data.append(np.hstack([q, pos, rpy_zyz]))

with open("cartesian_output_python.csv", "w") as f_stream:
    csv_writer = csv.writer(f_stream)
    csv_writer.writerows(output_data)

del ur_robot
del osc_controller
del inv_dyn_jnt_space
del fwd_dyn