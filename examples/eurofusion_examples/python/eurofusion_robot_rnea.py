import numpy as np
import csv
from numpy import genfromtxt
import sdu_controllers

input_trajectory = genfromtxt('../../data/breeder_trajectory_interpolated.csv', delimiter=',')

# Initialize robot model and parameters
bb_robot = sdu_controllers.BreedingBlanketHandlingRobotModel()
frequency = 1000.0
dt = 1.0 / frequency
ROBOT_DOF = bb_robot.get_dof()

rnea = sdu_controllers.RecursiveNewtonEuler(bb_robot)
z0 = np.array([0, 0, -1.0])
rnea.set_z0(z0)

q_d = np.zeros(ROBOT_DOF)
dq_d = np.zeros(ROBOT_DOF)
ddq_d = np.zeros(ROBOT_DOF)

q = np.array([0,0,0,0,0,3.1415,0])
dq = np.array([0,0,0,0,0,3.1415,0])
ddq = np.array([0,0,0,0,0,3.1415,0])

he = np.zeros(6)

tau = rnea.inverse_dynamics(q, dq, ddq, he)
print(tau)

output_data = []

t = 0.0
for point in input_trajectory:
  for i in range(0, ROBOT_DOF):
    q_d[i] = point[1 + i]
    dq_d[i] = point[1 + i + ROBOT_DOF]
    ddq_d[i] = point[1 + i +(2 * ROBOT_DOF)]

  tau = rnea.inverse_dynamics(q_d, dq_d, ddq_d, he)
  print('tau', tau)
  output_data.append(np.hstack([t, q_d, tau]))
  t += dt

with open("eurofusion_states.csv", "w") as f_stream:
    csv_writer = csv.writer(f_stream)
    csv_writer.writerows(output_data)
print('eurofusion_states.csv successfully created!')

del bb_robot
del rnea
