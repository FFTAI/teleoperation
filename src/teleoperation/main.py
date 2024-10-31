import datetime
import logging
import os
import time
from dataclasses import asdict
from pathlib import Path

import h5py
import hydra
import numpy as np
from omegaconf import DictConfig, OmegaConf

from teleoperation.data_collection import EpisodeDataDict, RecordingInfo
from teleoperation.filters import LPRotationFilter
from teleoperation.player import TeleopRobot
from teleoperation.state_machine import FSM
from teleoperation.utils import (
    CONFIG_DIR,
    RECORD_DIR,
    KeyboardListener,
    format_episode_id,
    se3_to_xyzortho6d,
)

logger = logging.getLogger(__name__)
OmegaConf.register_new_resolver("eval", eval, replace=True)

np.set_printoptions(precision=2, suppress=True)


class InitializationError(Exception):
    pass


PROJECT_ROOT = Path(__file__).resolve().parent.parent


@hydra.main(config_path=str(CONFIG_DIR), config_name="teleop_gr1", version_base="1.2")
def main(
    cfg: DictConfig,
):
    logger.info(f"Hydra output directory  : {hydra.core.hydra_config.HydraConfig.get().runtime.output_dir}")
    if not cfg.use_waist:
        cfg.robot.joints_to_lock.extend(cfg.robot.waist_joints)

    if not cfg.use_head:
        cfg.robot.joints_to_lock.extend(cfg.robot.head_joints)

    recording = None
    fsm = FSM()
    act = False

    data_dict = None
    session_name = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    if cfg.recording.enabled:
        session_path = RECORD_DIR / cfg.recording.task_name / session_name
        recording = RecordingInfo.from_session_path(str(session_path))
        data_dict = EpisodeDataDict.new(recording.episode_id, cfg.recording.camera_names)
        logger.info(f"Recording session: {session_path}")
        os.makedirs(session_path, exist_ok=True)

    robot = TeleopRobot(cfg)  # type: ignore

    listener = KeyboardListener()
    listener.start()

    def trigger():
        return listener.space_pressed

    head_filter = LPRotationFilter(cfg.head_filter.alpha)

    logger.info("Waiting for connection.")

    start_timer = None

    collection_start = None
    i = 0
    try:
        while True:
            start = time.monotonic()
            # ----- update readings -----
            (
                head_mat,
                left_wrist_mat,
                right_wrist_mat,
                left_qpos,
                right_qpos,
            ) = robot.step()

            head_mat = head_filter.next_mat(head_mat)

            robot.solve(left_wrist_mat, right_wrist_mat, head_mat, dt=1 / cfg.frequency)

            robot.set_hand_joints(left_qpos, right_qpos)

            if robot.viz and cfg.debug:
                robot.viz.viewer["head"].set_transform(head_mat)

            robot.update_display()

            # ----- logic -----
            if not robot.tv.connected:
                continue
            if robot.tv.connected and fsm.state == FSM.State.INITIALIZED:
                logger.info("Connected to headset.")
                fsm.next()
                start_timer = time.time()

            if start_timer is None:
                raise InitializationError("start_timer is None")

            if time.time() - start_timer < cfg.wait_time:
                logger.info(f"Waiting for trigger. Time elapsed: {time.time() - start_timer:.2f}")

            if fsm.state == FSM.State.STARTED and time.time() - start_timer > cfg.wait_time and trigger():
                logger.info("Trigger detected")

                fsm.next()
            elif fsm.state == FSM.State.CALIBRATING:
                logger.info("Calibrating.")
                # TODO: average over multiple frames
                robot.processor.calibrate(robot, head_mat, left_wrist_mat[:3, 3], right_wrist_mat[:3, 3])
                fsm.next()
            elif fsm.state == FSM.State.CALIBRATED:
                robot.init_control_joints()
                act = True
                fsm.next()
                continue

            elif fsm.state == FSM.State.ENGAGED and trigger():
                if not cfg.recording.enabled or recording is None:
                    logger.info("Disengaging.")
                    fsm.state = FSM.State.IDLE
                    robot.pause_robot()
                    continue
                fsm.next()

            elif fsm.state == FSM.State.IDLE and trigger():
                if not cfg.recording.enabled or recording is None:
                    logger.info("Engaging.")
                    fsm.state = FSM.State.ENGAGED
                    continue
                logger.info("Starting new episode.")
                fsm.next()

            elif fsm.state == FSM.State.EPISODE_STARTED:
                if not cfg.recording.enabled or recording is None:
                    raise InitializationError("Recording not initialized.")
                # collection_start = time.time()

                robot.start_recording(str(recording.video_path))

                data_dict = EpisodeDataDict.new(recording.episode_id, cfg.recording.camera_names)

                logger.info(f"Episode {recording.episode_id} started")
                fsm.next()
                continue
            elif fsm.state == FSM.State.COLLECTING and trigger():
                fsm.next()

            elif fsm.state == FSM.State.EPISODE_ENDED:
                if not cfg.recording.enabled or recording is None or data_dict is None:
                    raise InitializationError("Recording not initialized.")
                robot.pause_robot()

                # episode_length = time.time() - collection_start  # type: ignore

                logger.info(
                    f"Episode {recording.episode_id} took {data_dict.duration:.2f} seconds. Saving data to {recording.episode_path}"
                )

                # check for inhomogeneous data
                try:
                    with h5py.File(recording.episode_path, "w", rdcc_nbytes=1024**2 * 2) as f:
                        state = f.create_group("state")
                        action = f.create_group("action")

                        f.create_dataset("timestamp", data=data_dict.timestamp)

                        for name, data in asdict(data_dict.state).items():
                            state.create_dataset(name, data=np.asanyarray(data))

                        for name, data in asdict(data_dict.action).items():
                            action.create_dataset(name, data=np.asanyarray(data))

                        f.attrs["episode_id"] = format_episode_id(recording.episode_id)
                        f.attrs["task_name"] = str(cfg.recording.task_name)
                        f.attrs["camera_names"] = list(cfg.recording.camera_names)
                        f.attrs["episode_length"] = data_dict.length
                        f.attrs["episode_duration"] = data_dict.duration

                except Exception as e:
                    logger.error(f"Error saving episode: {e}")
                    import pickle

                    pickle.dump(data_dict, open(recording.episode_path + ".pkl", "wb"))
                    exit(1)

                recording.increment()
                robot.stop_recording()

                fsm.next()
                continue

            if act and (
                fsm.state == FSM.State.ENGAGED
                or fsm.state == FSM.State.EPISODE_STARTED
                or fsm.state == FSM.State.COLLECTING
            ):
                if not cfg.sim:
                    filtered_hand_qpos = robot.control_hands(left_qpos, right_qpos)
                    qpos = robot.control_joints()  # TODO: add gravity compensation

                    if fsm.state == FSM.State.COLLECTING and data_dict is not None:
                        data_dict.stamp()
                        left_pose = se3_to_xyzortho6d(left_wrist_mat)
                        right_pose = se3_to_xyzortho6d(right_wrist_mat)
                        head_pose = se3_to_xyzortho6d(head_mat)
                        data_dict.add_action(filtered_hand_qpos, qpos, np.hstack([left_pose, right_pose, head_pose]))

            if fsm.state == FSM.State.COLLECTING and data_dict is not None:
                qpos, hand_qpos, ee_pose, head_pose = robot.observe()

                data_dict.add_state(hand_qpos, qpos, np.hstack([ee_pose, head_pose]))
                i += 1

                # print("--------------------")
                # # print(f"head_euler: {np.rad2deg(head_euler)}")
                # print(f"head_trans: {head_mat[:3, 3]}")
                # print(f"left_pose: {left_pose}")
                # print(f"right_pose: {right_pose}")
                # print(f"left_qpos: {left_qpos}")
                # print(f"right_qpos: {right_qpos}")
                # print("--------------------")
                # print(data_dict)

            exec_time = time.monotonic() - start
            # logger.info(f"Execution time: {1/exec_time:.2f} hz")
            # print(max(0, 1 / config.frequency - exec_time))
            time.sleep(max(0, 1 / cfg.frequency - exec_time))

    except KeyboardInterrupt:
        robot.stop_recording()
        robot.end()

        time.sleep(1.0)
        exit(0)


if __name__ == "__main__":
    main()
