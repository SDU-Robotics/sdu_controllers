import roboticstoolbox as rtb
import numpy as np
import math
q_start = np.array([0.0, -math.pi/2, -math.pi/2, -math.pi/2, math.pi/2, 0])
q_goal = np.array([math.radians(45.0), math.radians(-120.0), -math.pi/2, math.radians(-60.0), math.pi/2, math.radians(-45.0)])
traj = rtb.jtraj(q_start, q_goal, 10)
print(traj.q)
print(traj.qd)
print(traj.qdd)
traj_comb = np.concatenate((traj.q, traj.qd, traj.qdd), axis=1)
np.savetxt("../examples/data/trajectory.csv", traj_comb, delimiter = ",")
