import numpy as np
import sdu_controllers


# Initialize robot model and parameters
bb_robot = sdu_controllers.models.BreedingBlanketHandlingRobotModel()
frequency = 1000.0
dt = 1.0 / frequency
ROBOT_DOF = bb_robot.get_dof()

q = np.array([0,0,0,0,0,3.1415,0])
dq = np.array([0,0,0,0,0,3.1415,0])
ddq = np.array([0,0,0,0,0,3.1415,0])

mass = 1e9
com = np.array([0., 0., 5.])
inertia = 1e5 * np.eye(3)

M = bb_robot.get_inertia_matrix(q)
print("M\n", M)

bb_robot.set_tcp_mass(mass, com, inertia)
M = bb_robot.get_inertia_matrix(q)
print("M\n", M)