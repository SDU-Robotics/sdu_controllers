#!/usr/bin/env python3
__all__ = (
    "models",
    "controllers",
    "math",
    "kinematics"
)

import numpy as _np

from ._sdu_controllers import models
from ._sdu_controllers import controllers

from ._sdu_controllers.controllers import PIDController as _original_PIDController

from ._sdu_controllers import math
from ._sdu_controllers import kinematics

# Export the version given in project metadata
from importlib import metadata

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

class _PIDController(_original_PIDController):
    def __init__(self, Kp, Ki, Kd, N, dt):
        args = [Kp, Ki, Kd, N]

        for i, arg in enumerate(args):
            if isinstance(arg, float):
                args[i] = _np.array([[arg]])

        super(_PIDController, self).__init__(*args, dt)

        self.output = _np.zeros((args[0].shape[0],))

    def step(self, q_d, dq_d, u_ff, q, dq):
        args = [q_d, dq_d, u_ff, q, dq]

        for i, arg in enumerate(args):
            if isinstance(arg, float):
                args[i] = _np.array([arg])

        super().step(*args)

        self.output = super().get_output()

controllers.PIDController = _PIDController