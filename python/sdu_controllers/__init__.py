from sdu_controllers._sdu_controllers import add_one
from sdu_controllers._sdu_controllers import RobotType
from sdu_controllers._sdu_controllers import URRobotModel
from sdu_controllers._sdu_controllers import BreedingBlanketHandlingRobotModel

from sdu_controllers._sdu_controllers import PIDController as PIDController_
from sdu_controllers._sdu_controllers import AdmittanceControllerPosition
from sdu_controllers._sdu_controllers import OperationalSpaceController

from sdu_controllers._sdu_controllers import InverseDynamicsJointSpace
from sdu_controllers._sdu_controllers import ForwardDynamics
from sdu_controllers._sdu_controllers import RecursiveNewtonEuler

from sdu_controllers._sdu_controllers import forward_kinematics

# Export the version given in project metadata
from importlib import metadata
import numpy as np

__version__ = metadata.version(__package__)

del metadata

# def one_plus_one():
 #    return add_one(1)

# def PIDController(Kp, Ki, Kd, N, dt):
#     print("PID")
#     if isinstance(Kp, float):
#         Kp = np.array([[Kp]])
#
#     if isinstance(Ki, float):
#         Ki = np.array([[Ki]])
#
#     if isinstance(Kd, float):
#         Kd = np.array([[Kd]])
#
#     if isinstance(N, float):
#         N = np.array([[N]])
#
#     return PIDController_(Kp, Ki, Kd, N, dt)

class PIDController(PIDController_):
    def __init__(self, Kp, Ki, Kd, N, dt):
        args = [Kp, Ki, Kd, N]

        for i, arg in enumerate(args):
            if isinstance(arg, float):
                args[i] = np.array([[arg]])

        super().__init__(*args, dt)

        self.output = np.zeros((args[0].shape[0],))

    def step(self, q_d, dq_d, u_ff, q, dq):
        args = [q_d, dq_d, u_ff, q, dq]

        for i, arg in enumerate(args):
            if isinstance(arg, float):
                args[i] = np.array([arg])

        super().step(*args)

        self.output = super().get_output()