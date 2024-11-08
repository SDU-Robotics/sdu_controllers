import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import math
import matplotlib.pyplot as plt

frequency = 500.0
dt = 1 / frequency

t = np.arange(0, 4, dt)
TE1 = sm.SE3.Trans(0.4918, -0.13336, 0.4878) * sm.SE3.Rx(3)
TE2 = sm.SE3.Trans(0.3995, -0.2453, 0.3857) * sm.SE3.Rx(1)

cart_traj = rtb.ctraj(TE1, TE2, t)
x_prev = np.zeros(6)
dx_prev = np.zeros(6)
pos = []
vel = []
acc = []

for pose in cart_traj:
    # Positional part
    x = np.concatenate([pose.t, sm.base.tr2eul(pose.R)]) # pose as Euler angles ZYZ
    print('x:', x)
    dx = (x_prev - x) / dt
    print('dx:', dx)
    ddx = (dx_prev - dx) / dt
    print('ddx:', ddx)
    pos.append(x)
    vel.append(dx)
    acc.append(ddx)

    x_prev = x
    dx_prev = dx

#rtb.xplot(t, np.array(pos), labels="x y z r p y")
#rtb.xplot(t, np.array(vel), labels="vel_x vel_y vel_z omega_x omega_y omega_z")
#rtb.xplot(t, np.array(acc), labels="acc_x acc_y acc_z domega_x domega_y domega_z")
#plt.show()

traj_comb = np.concatenate((np.array(pos), np.array(vel), np.array(acc)), axis=1)
np.savetxt("../examples/data/cartesian_trajectory_safe.csv", traj_comb, delimiter =",")
