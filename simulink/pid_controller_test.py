import time

import sdu_controllers as sc
import numpy as np
import matplotlib.pyplot as plt

##
# Kp = np.array([[20.]])
# Kd = np.array([[5.]])
# Ki = np.array([[20.]])
# N = np.array([[0.]])
Kp = 20.
Kd = 5.
Ki = 20.
N = 0.

dt = 1e-4

pid = sc.PIDController(Kp, Ki, Kd, N, dt)
print(pid)

tvec = np.arange(0, 20, dt)

print(tvec)

###

q_d = np.sin(0.1 * tvec)
dq_d = np.zeros([tvec.size,])
u_ff = np.zeros([tvec.size,])

ddq = np.zeros([tvec.size,])
dq = np.zeros([tvec.size,])
q = np.zeros([tvec.size,])

print(tvec.size)
#
t0 = time.time()
for i in range(tvec.size - 1):
    q_d_np = q_d[i]
    dq_d_np = dq_d[i]
    u_ff_np = u_ff[i]
    q_np = q[i]
    dq_np = dq[i]

    pid.step(q_d_np, dq_d_np, u_ff_np, q_np, dq_np)
    y = pid.get_output()
    # y = np.asarray(y)
    # print(y)

    ddq[i] = 10 * q[i] - 5 * dq[i] + y
    dq[i + 1] = dq[i] + dt * ddq[i]
    q[i + 1] = q[i] + dt * dq[i]

t1 = time.time()

# print(q)
print(f"Duration {t1 - t0}")

plt.figure()
plt.plot(tvec, q)
plt.plot(tvec, q_d)


plt.show()
