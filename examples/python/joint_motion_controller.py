import numpy as np
from numpy import genfromtxt
#from sdu_controllers import RobotType
#from sdu_controllers import URRobotModel
#from sdu_controllers import PDController
#from sdu_controllers import InverseDynamicsJointSpace
import sdu_controllers

my_data = genfromtxt('examples/data/trajectory_safe.csv', delimiter=',')

Kp_val = 100.0
Kd_val = 2 * np.sqrt(Kp_val)
N_val = 1

Kp = np.diag([Kp_val, Kp_val, Kp_val])
Kd = np.diag([Kd_val, Kd_val, Kd_val])
N = np.diag([N_val, N_val, N_val])

ur_robot = sdu_controllers.URRobotModel()
pd_controller = sdu_controllers.PDController(Kp, Kd, N)
inv_dyn_jnt_space = sdu_controllers.InverseDynamicsJointSpace(ur_robot)

q = np.array([0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0])
dq = np.zeros(6)

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

input("Press enter to quit!")
del ur_robot
del pd_controller
del inv_dyn_jnt_space

