import sys

import numpy as np
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_control import RTDEControlInterface as RTDEControl
import time
import sdu_controllers
import copy

from pytransform3d.transform_manager import TransformManager
from pytransform3d import transformations as pt
from pytransform3d import rotations as pr

def get_transform_as_ur_pose(transform):
    T_base_tcp_out_pq = pt.pq_from_transform(transform)
    T_base_tcp_out_pos = T_base_tcp_out_pq[0:3]
    T_base_tcp_out_rotvec = pr.compact_axis_angle_from_quaternion(T_base_tcp_out_pq[3:7])
    return np.concatenate((T_base_tcp_out_pos, T_base_tcp_out_rotvec))

def get_transform_as_pq(tm, from_frame, to_frame):
    T_base_tip = tm.get_transform(from_frame, to_frame)
    T_base_tip_pq = pt.pq_from_transform(T_base_tip)
    T_base_tip_pos = T_base_tip_pq[0:3]
    T_base_tip_quat = T_base_tip_pq[3:7]
    return T_base_tip_pos, T_base_tip_quat

def skew_symmetric(vector):
    x = vector[0]
    y = vector[1]
    z = vector[2]
    Sv = np.zeros((3, 3))
    Sv[1, 0] = z
    Sv[2, 0] = -y
    Sv[0, 1] = -z
    Sv[2, 1] = x
    Sv[0, 2] = y
    Sv[1, 2] = -x
    return Sv

def adjoint(matrix):
    # Assumes input is 4x4 transformation matrix
    R_mat = matrix[0:3, 0:3]
    p = matrix[0:3, 3]

    adj_T = np.zeros((6, 6))
    adj_T[0:3, 0:3] = R_mat
    adj_T[3:6, 0:3] = skew_symmetric(p) @ R_mat
    adj_T[3:6, 3:6] = R_mat

    return adj_T

def wrench_trans(torques, forces, T):
    wrench_in_A = np.hstack((torques, forces))
    wrench_in_B = adjoint(T).T @ wrench_in_A
    return wrench_in_B[:3], wrench_in_B[3:]

def get_robot_pose_as_transform(pose):
    pq = np.hstack([pose[0:3], pr.quaternion_from_compact_axis_angle(pose[3:6])])
    T_base_tcp = pt.transform_from_pq(pq)
    return T_base_tcp


def get_circle_target(pose, timestep, radius=0.075, freq=0.5):
    circ_target = copy.deepcopy(pose)
    circ_target[0] = pose[0] + radius * np.cos((2 * np.pi * freq * timestep))
    circ_target[1] = pose[1] + radius * np.sin((2 * np.pi * freq * timestep))
    return circ_target


def main():
    robot_ip = sys.argv[1]

    frequency = 500.0
    dt = 1/frequency

    rtde_receive = RTDEReceive(robot_ip)
    rtde_control = RTDEControl(robot_ip)

    rtde_control.zeroFtSensor()
    time.sleep(0.2)

    adm_controller = sdu_controllers.AdmittanceControllerPosition(frequency)
    adm_controller.set_mass_matrix_position(np.identity(3) * 22.5)
    adm_controller.set_stiffness_matrix_position(np.identity(3) * 54)
    adm_controller.set_damping_matrix_position(np.identity(3) * 65)

    adm_controller.set_mass_matrix_orientation(np.identity(3) * 0.25)
    adm_controller.set_stiffness_matrix_orientation(np.identity(3) * 10)
    adm_controller.set_damping_matrix_orientation(np.identity(3) * 5)

    # Initialize transform manager
    tm = TransformManager()

    T_base_tcp = get_robot_pose_as_transform(rtde_receive.getActualTCPPose())

    # Define tip
    T_tcp_tip = pt.transform_from_pq(np.hstack((np.array([0, 0, 0.12]), np.array([1.0, 0.0, 0.0, 0.0]))))
    T_tip_tcp = np.linalg.inv(T_tcp_tip)

    # add the transforms to the transform manager
    tm.add_transform("TCP", "Base", T_base_tcp)
    tm.add_transform("tip", "TCP", T_tcp_tip)

    T_base_tip_pos_init, T_base_tip_quat_init = get_transform_as_pq(tm, "tip", "Base")

    # Set target circle
    counter = 0.0
    x_desired = get_circle_target(T_base_tip_pos_init, counter)

    T_base_tip_circle = pt.transform_from_pq(np.hstack((x_desired, T_base_tip_quat_init)))
    T_base_tcp_circle = T_base_tip_circle @ T_tip_tcp

    # Use moveL to move to the initial point on the circle.
    rtde_control.moveL(get_transform_as_ur_pose(T_base_tcp_circle))

    try:
        while True:
            start_time = rtde_control.initPeriod()

            # update the transform in the transform manager
            T_base_tcp = get_robot_pose_as_transform(rtde_receive.getActualTCPPose())
            tm.add_transform("TCP", "Base", T_base_tcp)

            ft = rtde_receive.getActualTCPForce()
            f_base, mu_base = ft[0:3], ft[3:7]

            # rotate forces from base frame to TCP frame (necessary on a UR robot)
            R_tcp_base = tm.get_transform("Base", "TCP")[:3, :3]
            f_tcp = R_tcp_base @ f_base
            mu_tcp = R_tcp_base @ mu_base

            # use wrench transform to place the force torque in the tip.
            mu_tip, f_tip = wrench_trans(mu_tcp, f_tcp, tm.get_transform("tip", "TCP"))

            # rotate forces back to base frame
            R_base_tip = tm.get_transform("tip", "Base")[:3, :3]
            f_base_tip = R_base_tip @ f_tip

            x_desired = get_circle_target(T_base_tip_pos_init, counter)

            adm_controller.step(f_base_tip, mu_tip, x_desired, T_base_tip_quat_init)
            output = adm_controller.get_output()

            # rotate output from tip to TCP before sending it to the robot
            T_base_tip_out = pt.transform_from_pq(np.hstack((output[0:3], output[3:7])))
            T_base_tcp_out = T_base_tip_out @ tm.get_transform("TCP", "tip")
            base_tcp_out_ur_pose = get_transform_as_ur_pose(T_base_tcp_out)

            rtde_control.servoL(base_tcp_out_ur_pose, 0.0, 0.0, dt, 0.03, 2000)
            rtde_control.waitPeriod(start_time)

            counter = counter + dt

    except KeyboardInterrupt:
        print("Control interrupted")

    # Shutdown
    rtde_control.servoStop(10.0)

    rtde_receive.disconnect()
    rtde_control.disconnect()


if __name__ == "__main__":
    main()
