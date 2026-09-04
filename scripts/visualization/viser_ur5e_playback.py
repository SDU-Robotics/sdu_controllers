"""Play back a joint-space trajectory log on a UR5e in Viser for recording videos.

This is a *visualization-only* helper - it is not part of the sdu_controllers
library or its tests. It reads a CSV log produced by one of the simulation
examples (e.g. examples/simulation/ur/cpp/impedance_controller.cpp or
examples/simulation/ur/python/impedance_controller.py, which both log the
joint positions q0..q5 per row) and animates the corresponding UR5e URDF in a
Viser scene, so the motion can be screen-recorded into a short video.

Requires the optional dependencies listed in requirements.txt in this folder:

    pip install -r scripts/visualization/requirements.txt

Usage:

    python scripts/visualization/viser_ur5e_playback.py output_impedance.csv \
        --q-columns 1 2 3 4 5 6 --fps 60 --control-rate 500

The script loads a logged joint trajectory, renders the UR5e in a Viser browser
window, and waits for the user to press the "Start playback" button before
playing the motion. It resets to the initial joint configuration after each run.
"""

import argparse
import threading
import time

import numpy as np
import viser
from robot_descriptions.loaders.yourdfpy import load_robot_description
from viser.extras import ViserUrdf


def load_joint_log(csv_path: str, q_columns: list[int]) -> np.ndarray:
    """Load joint positions (radians) from a CSV log.

    :param csv_path: path to the CSV log file.
    :param q_columns: 0-indexed column numbers containing q0..qN, in order.
    :returns: array of shape (num_steps, len(q_columns)).
    """
    data = np.genfromtxt(csv_path, delimiter=",")
    return data[:, q_columns]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_path", help="CSV log containing a joint-position trajectory.")
    parser.add_argument(
        "--q-columns", type=int, nargs=6, default=[0, 1, 2, 3, 4, 5],
        help="0-indexed CSV columns holding q0..q5 (default: 0 1 2 3 4 5).")
    parser.add_argument("--fps", type=float, default=30.0, help="Playback frame rate.")
    parser.add_argument(
        "--control-rate", type=float, default=500.0,
        help="Rate [Hz] at which the CSV rows were logged (default: 500).")
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="Playback speed relative to real time (default: 1.0).")
    parser.add_argument("--loop", action="store_true", help="Loop the playback indefinitely.")
    args = parser.parse_args()

    joint_traj = load_joint_log(args.csv_path, args.q_columns)

    # The log is sampled at the control rate, so only a subset of rows falls on a display frame.
    num_rows = joint_traj.shape[0]
    playback_duration = num_rows / args.control_rate / args.speed
    frame_times = np.arange(0.0, playback_duration, 1.0 / args.fps)
    frame_rows = np.clip(
        np.round(frame_times * args.speed * args.control_rate).astype(int), 0, num_rows - 1)

    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=3, height=3, cell_size=0.1)
	# Apply ROS-vs-real base convention correction: 180 deg yaw around base Z.
    server.scene.add_frame("/robot_base", wxyz=(0.0, 0.0, 0.0, 1.0), position=(0.0, 0.0, 0.0), show_axes=False)

    @server.on_client_connect
    def _(client: viser.ClientHandle) -> None:
        client.camera.position = (1.020,0.590,0.670)
        client.camera.look_at = (0.119,-0.197,0.346)
        client.camera.fov = 1.3090
        client.camera.far = 1000

    urdf = load_robot_description("ur5e_description")
    viser_urdf = ViserUrdf(server, urdf, root_node_name="/robot_base/robot")

    actuated_joints = viser_urdf.get_actuated_joint_names()
    if joint_traj.shape[1] != len(actuated_joints):
        parser.error(
            f"CSV provides {joint_traj.shape[1]} joint columns, but the URDF has "
            f"{len(actuated_joints)} actuated joints: {actuated_joints}")

    viser_urdf.update_cfg(joint_traj[0])

    start_button = server.gui.add_button("Start playback")
    playback_requested = threading.Event()

    @start_button.on_click
    def _(_) -> None:
        playback_requested.set()

    print(f"Loaded {num_rows} frames from {args.csv_path}.")
    print(f"Playing back {len(frame_rows)} frames at {args.fps:g} fps ({playback_duration:.1f} s).")
    print("Open the printed URL in a browser, press 'Start playback', then screen-record the animation.")

    frame_period = 1.0 / args.fps
    while True:
        playback_requested.wait()
        playback_requested.clear()
        start_button.disabled = True

        start_time = time.perf_counter()
        for frame, row in enumerate(frame_rows):
            viser_urdf.update_cfg(joint_traj[row])
            # Schedule against the start time so dropped frames do not accumulate lag.
            time.sleep(max(0.0, start_time + (frame + 1) * frame_period - time.perf_counter()))

        viser_urdf.update_cfg(joint_traj[0])
        start_button.disabled = False


if __name__ == "__main__":
    main()
