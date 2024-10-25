import roboticstoolbox as rtb
import numpy as np
import math
frequency = 500.0
dt = 1 / frequency
q_start = np.array([0.0, -math.pi/2, -math.pi/2, -math.pi/2, math.pi/2, 0])
q_goal = np.array([math.radians(45.0), math.radians(-120.0), -math.pi/2, math.radians(-60.0), math.pi/2, math.radians(-45.0)])
velocity = 1.05
max_dist = 0.0
for i in range(0, 6):
    max_dist = max(max_dist, math.fabs(q_start[i] - q_goal[i]))
trajectory_duration = max_dist / velocity
trajectory_samples = int(trajectory_duration * int(1 / dt))
t_array = np.linspace(0.0, trajectory_duration, num=trajectory_samples)

traj = rtb.jtraj(q_start, q_goal, t_array)
print(traj.q)
print(traj.qd)
print(traj.qdd)
traj_comb = np.concatenate((traj.q, traj.qd, traj.qdd), axis=1)
np.savetxt("../examples/data/trajectory_safe.csv", traj_comb, delimiter = ",")
