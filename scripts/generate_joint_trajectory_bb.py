import roboticstoolbox as rtb
import numpy as np
import math
import csv

frequency = 1000.0
dt = 1 / frequency
robot_dof = 7
# q_start = np.array([0.0, -math.pi/2, -math.pi/2, -math.pi/2, math.pi/2, 0])
# q_goal = np.array([math.radians(45.0), math.radians(-120.0), -math.pi/2, math.radians(-60.0), math.pi/2, math.radians(-45.0)])
# velocity = 1.05
# max_dist = 0.0
# for i in range(0, 6):
#     max_dist = max(max_dist, math.fabs(q_start[i] - q_goal[i]))
# trajectory_duration = max_dist / velocity
# trajectory_samples = int(trajectory_duration * int(1 / dt))
# t_array = np.linspace(0.0, trajectory_duration, num=trajectory_samples)
trajectory_in = []

with open('../examples/data/eurofusion_robot_trajectory.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        q = []
        for i in range(0, robot_dof):
            q.append(float(row[1+i]))
        trajectory_in.append(np.array(q))

viapoints = np.array(trajectory_in)
trajectory = []
qd_prev = np.zeros(7)

for i in range(1, len(viapoints)):
    via_traj = rtb.jtraj(viapoints[i-1], viapoints[i], t=1000, qd0=qd_prev)
    via_traj_comb = np.concatenate((via_traj.q, via_traj.qd, via_traj.qdd), axis=1)
    trajectory.append(via_traj_comb)
    qd_prev = via_traj.qd[-1]

trajectory_np = np.vstack(trajectory)

time = 0.0
time_vec = []

for i in range(0, len(trajectory_np)):
    time_vec.append(time)
    time += dt

traj_comb = np.concatenate((np.vstack(np.array(time_vec)), trajectory_np), axis=1)

print(traj_comb)
np.savetxt("../examples/data/joint_trajectory_safe_bb.csv", traj_comb, delimiter =",")
