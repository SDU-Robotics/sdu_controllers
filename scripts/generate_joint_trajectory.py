import roboticstoolbox as rtb
import numpy as np
import math
frequency = 500.0
dt = 1 / frequency
q_start = np.array([0.0, -math.pi/2, -math.pi/2, -math.pi/2, math.pi/2, 0])
q_goal = np.array([math.radians(45.0), math.radians(-120.0), -math.pi/2, math.radians(-60.0), math.pi/2, math.radians(-45.0)])
duration_in_seconds = 3.0

traj = rtb.jtraj(q_start, q_goal, t = int(duration_in_seconds * frequency))
print(traj.q)
print(traj.qd)
print(traj.qdd)
traj_comb = np.concatenate((traj.q, traj.qd, traj.qdd), axis=1)
np.savetxt("../examples/data/joint_trajectory_safe.csv", traj_comb, delimiter =",")
