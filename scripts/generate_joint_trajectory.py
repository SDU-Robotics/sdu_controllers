import roboticstoolbox as rtb
import numpy as np
import math
frequency = 500.0
dt = 1 / frequency
duration_in_seconds = 6.0

viapoints = np.array([[-math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, math.pi/2, 0],
  [math.radians(-120.0), math.radians(-120.0), math.radians(-60.0), math.radians(-60.0), math.radians(45.0), math.radians(-45.0)],
  [-math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, math.pi/2, 0]])
trajectory = []
qd_prev = np.zeros(6)

for i in range(1, len(viapoints)):
    via_traj = rtb.jtraj(viapoints[i-1], viapoints[i], t=int(duration_in_seconds * frequency), qd0=qd_prev)
    #print(via_traj.q)
    #print(via_traj.qd)
    #print(via_traj.qdd)
    via_traj_comb = np.concatenate((via_traj.q, via_traj.qd, via_traj.qdd), axis=1)
    trajectory.append(via_traj_comb)
    qd_prev = via_traj.qd[-1]

trajectory_np = np.vstack(trajectory)
np.savetxt("../data/joint_trajectory_safe_new.csv", trajectory_np, delimiter =",")



#q_start = np.array([-math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, math.pi/2, 0])
#q_goal = np.array([math.radians(-120.0), math.radians(-120.0), math.radians(-60.0), math.radians(-60.0), math.radians(45.0), math.radians(-45.0)])
#duration_in_seconds = 3.0

#traj = rtb.jtraj(q_start, q_goal, t = int(duration_in_seconds * frequency))
#print(traj.q)
#print(traj.qd)
#print(traj.qdd)
#traj_comb = np.concatenate((traj.q, traj.qd, traj.qdd), axis=1)
#np.savetxt("../data/joint_trajectory_safe.csv", traj_comb, delimiter =",")
