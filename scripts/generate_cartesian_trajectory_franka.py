import swift
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import math
from telnetlib import AO

def get_circle_target(timestep, radius=0.075, freq=0.33):
    delta_x = radius * np.cos((2 * np.pi * freq * timestep))
    delta_y = radius * np.sin((2 * np.pi * freq * timestep))
    return delta_x, delta_y

env = swift.Swift()
env.launch(realtime=True)

panda = rtb.models.Panda()
panda.q = panda.qr
q_init = np.array([0.0, -math.pi/4, 0, -3.0 * math.pi/4, 0, math.pi/2, math.pi/4])
T_init = panda.fkine(q=q_init)

arrived = False
env.add(panda)

frequency = 1000.0  # Hz
dt = 1.0 / frequency
t = 0.0
time_vec = []
traj = []

while t < 10.0:
    t += dt
    delta_x, delta_y = get_circle_target(t)
    T_delta = sm.SE3.Trans(delta_x, delta_y, 0.0) * sm.SE3()
    T_new = T_init @ T_delta.inv()
    pose = np.concatenate((T_new.t, T_new.eulervec()))
    time_vec.append(np.array([t]))
    traj.append(pose)

    arrived = False
    while not arrived:
      v, arrived = rtb.p_servo(panda.fkine(q=panda.q), T_new, 1.5, 0.1)
      panda.qd = np.linalg.pinv(panda.jacobe(q=panda.q)) @ v
      env.step(dt)

env.close()
traj_comb = np.concatenate((np.array(time_vec), np.array(traj)), axis=1)
np.savetxt("../examples/data/franka_cartesian_trajectory_circle.csv", traj_comb, delimiter =",")

# import roboticstoolbox as rtb
# import spatialmath as sm
# from spatialmath import base as smb
# import numpy as np
# import math
# import matplotlib.pyplot as plt
# from spatialmath.base.symbolic import symbol

# robot = rtb.models.DH.UR5()

# frequency = 500.0
# dt = 1 / frequency

# t = np.arange(0, 4, dt)
# #TE1 = sm.SE3.Trans([0.4918, -0.13336, 0.4878]) * sm.SE3.RPY([math.pi/4, math.pi/4, math.pi/4], unit="rad", order="zyx")
# #TE2 = sm.SE3.Trans([0.258, -0.263, 0.395]) * sm.SE3.RPY([math.pi/4, math.pi/4, math.pi/4], unit="rad", order="zyx")
# TE1 = robot.fkine(q=np.array([0.0, -1.5707, -1.5707, -1.5707, 1.5707, 0.0]))
# #print(TE1, sm.base.tr2eul(TE1.R, check=True, flip=True))
# #exit()
# TE2 = robot.fkine(q=np.array([math.pi/4, -1.5707, -1.5707, -1.5707, 1.5707-(math.pi/4), math.pi/4]))
# #TE1 = sm.SE3.Trans(0.4918, -0.13336, 0.4878) * sm.SE3.EulerVec(np.array([0.0006426083866161601, -3.1364574763614557, -0.03915484228919635]))
# #TE2 = sm.SE3.Trans(0.3995, -0.2453, 0.3857) * sm.SE3.EulerVec(np.array([-0.0058207991671794254, -2.771465145790119, -0.03471883722219153]))

# print('TE1:', TE1.t, smb.tr2eul(TE1.R, flip=True, check=True))
# print('TE1 rpy:',  smb.tr2rpy(TE1.R, order="xyz", check=True))
# print('TE2:', TE2.t, smb.tr2eul(TE2.R, flip=True, check=True))
# print('TE2 rot mat:', TE2.R)
# print('TE2 rpy:',  smb.base.tr2rpy(TE2.R, order="xyz", check=True))
# cart_traj = rtb.ctraj(T0=TE1, T1=TE2, t=t)
# x_prev = np.concatenate([TE1.t, smb.tr2eul(TE1.R, flip=True, check=True)])
# dx_prev = np.zeros(6)
# pos = []
# vel = []
# acc = []

# for pose in cart_traj:
#     x = np.concatenate([pose.t, smb.tr2eul(pose.R, flip=True, check=True)]) # pose as Euler angles ZYZ
#     #print('x:', x)
#     dx = (x_prev - x) / dt
#     ddx = (dx_prev - dx) / dt

#     pos.append(x)
#     vel.append(dx)
#     acc.append(ddx)

#     x_prev = x
#     dx_prev = dx

# rtb.xplot(t, np.array(pos), labels="x y z r p y")
# rtb.xplot(t, np.array(vel), labels="vel_x vel_y vel_z omega_x omega_y omega_z")
# rtb.xplot(t, np.array(acc), labels="acc_x acc_y acc_z domega_x domega_y domega_z")
# plt.show()

# traj_comb = np.concatenate((np.array(pos), np.array(vel), np.array(acc)), axis=1)
# np.savetxt("../examples/data/cartesian_trajectory_safe.csv", traj_comb, delimiter =",")
# print("successfully saved ../examples/data/cartesian_trajectory_safe.csv")
