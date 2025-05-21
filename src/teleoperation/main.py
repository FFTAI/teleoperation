import datetime
import logging
import os
import time
from pathlib import Path

import hydra
import numpy as np
import psutil
from omegaconf import DictConfig

from teleoperation.data_collection import EpisodeDataDict, RecordingInfo, get_camera_names
from teleoperation.filters import LPRotationFilter
from teleoperation.player import TeleopRobot
from teleoperation.state_machine import FSM
from teleoperation.utils import (
    CONFIG_DIR,
    RECORD_DIR,
    KeyboardListener,
    se3_to_xyzortho6d,
    so3_to_ortho6d,
)

logger = logging.getLogger(__name__)


class InitializationError(Exception):
    pass


PROJECT_ROOT = Path(__file__).resolve().parent.parent


@hydra.main(config_path=str(CONFIG_DIR), config_name="teleop", version_base="1.2")
def main(
    cfg: DictConfig,
):
    p = psutil.Process()
    p.cpu_affinity(cfg.cpu.affinity)
    try:
        p.nice(cfg.cpu.niceness)
    except psutil.AccessDenied:
        logger.info(f"Failed to set niceness {cfg.cpu.niceness}, not a privileged user.")

    logger.info(f"Setting CPU affinity: {cfg.cpu.affinity}; Niceness: {cfg.cpu.niceness}")

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

    camera_names = get_camera_names(cfg)

    logger.info(f"Camera names: {camera_names}")

    if cfg.recording.enabled:
        session_path = RECORD_DIR / cfg.recording.task_name / session_name
        recording = RecordingInfo.from_session_path(str(session_path))
        data_dict = EpisodeDataDict.new(recording.episode_id, camera_names)
        logger.info(f"Recording enabled. Creating recording session at {session_path}.")
        logger.info(f"Operator: {cfg.recording.operator}; Pilot: {cfg.recording.pilot}")
        os.makedirs(session_path, exist_ok=True)

    robot = TeleopRobot(cfg)  # type: ignore

    listener = KeyboardListener()
    listener.start()

    def make_debounce_trigger(debounce_tol):
        """Debounce trigger"""
        last_trigger = time.time()

        def _trigger():
            """Get keyboard triggers. `space` for FSM state transition, `x` for discarding current episode, `s` for stop (TODO)"""
            pressed = listener.key_pressed
            # logger.info(f"Pressed keys: {pressed}")
            if pressed is None:
                return False, None
            if pressed.get("space", False):
                return True, "space"
            for key, value in pressed.items():
                if value and key in ["q", "x", "d", "s", "z", "p"]:
                    return True, key
            return False, None

        def trigger():
            nonlocal last_trigger
            triggered, key = _trigger()
            if triggered and time.time() - last_trigger > debounce_tol:
                last_trigger = time.time()
                return triggered, key
            elif triggered:
                logger.warning(f"Debouncing trigger: {key}")
            return False, None

        return trigger

    trigger = make_debounce_trigger(0.5)

    def trigger_key(key):
        return listener.key_pressed.get(key, False)

    head_filter = LPRotationFilter(cfg.head_filter.alpha)

    logger.info("Waiting for connection.")

    start_timer = None

    collection_start = None
    i = 0
    try:
        while True:
            start = time.monotonic()
            # step robot states
            (
                head_mat,
                left_wrist_mat,
                right_wrist_mat,
                left_qpos,
                right_qpos,
            ) = robot.step()

            head_mat = head_filter.next_mat(head_mat)

            # set new ik target for left/right wrist and head orientation,
            # the result will be updated to robot.configuration,
            # which can be retrieved from property robot.q_real
            # and actuated by calling robot.control_joints()
            robot.solve(left_wrist_mat, right_wrist_mat, head_mat, dt=1 / cfg.frequency)

            # set hand joints detected from headset to the internal model, mainly for display
            robot.set_hand_joints(left_qpos, right_qpos)

            if robot.viz and cfg.debug:
                # display head target
                robot.viz.viewer["head"].set_transform(head_mat)

            # call this to update meshcat visualization, does nothing if visualization is not enabled
            robot.update_display()

            # ----- logic -----
            # skip if cionnection to headset not established
            if not robot.tv.connected:
                continue
            if robot.tv.connected and fsm.state == FSM.State.INITIALIZED:
                logger.info("Connected to headset.")
                fsm.state = FSM.State.STARTED
                start_timer = time.time()

            if start_timer is None:
                raise InitializationError("start_timer is None")

            if time.time() - start_timer < cfg.wait_time:
                logger.info(f"Waiting for trigger. Time elapsed: {time.time() - start_timer:.2f}")

            # get trigger: space for continue, x for discard, s for stop
            # print help message
            trigger_detected, triggered_key = trigger()
            if trigger_detected:
                logger.debug(f"Trigger detected: {trigger_detected}, key: {triggered_key}")

            if (
                fsm.state == FSM.State.STARTED
                and time.time() - start_timer > cfg.wait_time
                and triggered_key == "space"
            ):
                logger.info("Trigger detected")

                fsm.state = FSM.State.CALIBRATING
            elif fsm.state == FSM.State.CALIBRATING:
                logger.info("Calibrating.")
                # TODO: average over multiple frames
                robot.processor.calibrate(robot, head_mat, left_wrist_mat[:3, 3], right_wrist_mat[:3, 3])
                fsm.state = FSM.State.CALIBRATED
            elif fsm.state == FSM.State.CALIBRATED:
                robot.init_control_joints()
                act = True
                fsm.state = FSM.State.ENGAGED
                continue

            elif fsm.state == FSM.State.ENGAGED and triggered_key == "space":
                if not cfg.recording.enabled or recording is None:
                    logger.info("Disengaging.")
                    fsm.state = FSM.State.IDLE
                    robot.pause_robot()
                    continue
                else:
                    fsm.state = FSM.State.EPISODE_STARTED
                    logger.info("Starting new episode.")

            elif fsm.state == FSM.State.IDLE and triggered_key == "space":
                if not cfg.recording.enabled or recording is None:
                    logger.info("Engaging.")
                    fsm.state = FSM.State.ENGAGED
                    continue
                else:
                    fsm.state = FSM.State.EPISODE_STARTED
                    logger.info("Starting new episode.")

            elif (
                fsm.state == FSM.State.IDLE and triggered_key == "x" and cfg.recording.enabled and recording is not None
            ):
                fsm.state = FSM.State.ENGAGED
                logger.info("Re-engaging...")

            elif fsm.state == FSM.State.EPISODE_STARTED:
                if not cfg.recording.enabled or recording is None:
                    raise InitializationError("Recording not initialized.")
                # collection_start = time.time()
                recording.acquire()
                robot.start_recording(str(recording.video_path))

                data_dict = EpisodeDataDict.new(recording.episode_id, camera_names)

                logger.info(f"Episode {recording.episode_id} started")
                fsm.state = FSM.State.COLLECTING
                continue
            elif fsm.state == FSM.State.COLLECTING:
                if triggered_key == "space":
                    fsm.state = FSM.State.EPISODE_ENDED
                elif triggered_key == "x":
                    fsm.state = FSM.State.EPISODE_DISCARDED

            elif fsm.state == FSM.State.EPISODE_DISCARDED:
                if not cfg.recording.enabled or recording is None:
                    raise InitializationError("Recording not initialized.")
                logger.warning(f"Episode {recording.episode_id} discarded")
                data_dict = None
                robot.stop_recording()

                recording.release()

                time.sleep(0.5)
                _, _ = trigger()

                fsm.state = FSM.State.IDLE
                continue

            elif fsm.state == FSM.State.EPISODE_ENDED:
                if not cfg.recording.enabled or recording is None or data_dict is None:
                    raise InitializationError("Recording not initialized.")
                robot.pause_robot()
                recording.save_episode(data_dict, cfg)
                robot.stop_recording()

                # episode_length = time.time() - collection_start  # type: ignore
                logger.info(
                    f"Episode {recording.episode_id} took {data_dict.duration:.2f} seconds. Saving data to {recording.episode_path}"
                )

                time.sleep(0.5)

                data_dict = None
                recording.release()
                recording.increment()

                _, _ = trigger()

                fsm.state = FSM.State.IDLE
                continue

            if act and (
                fsm.state == FSM.State.ENGAGED
                or fsm.state == FSM.State.EPISODE_STARTED
                or fsm.state == FSM.State.COLLECTING
            ):
                # TODO: this is not actually filtered tho
                filtered_hand_qpos = robot.control_hands(left_qpos, right_qpos, return_real=False)
                qpos = robot.control_joints()

                if fsm.state == FSM.State.COLLECTING and data_dict is not None:
                    data_dict.stamp()
                    left_pose = se3_to_xyzortho6d(left_wrist_mat)
                    right_pose = se3_to_xyzortho6d(right_wrist_mat)
                    head_pose = so3_to_ortho6d(head_mat)

                    data_dict.add_action(filtered_hand_qpos, qpos, np.hstack([left_pose, right_pose, head_pose]))

            if fsm.state == FSM.State.COLLECTING and data_dict is not None:
                qpos, hand_qpos, ee_pose, head_pose = robot.observe()

                if cfg.hand.get("use_tactile", False):
                    left_tactile, right_tactile = robot.left_hand.get_tactile(), robot.right_hand.get_tactile()
                    # logger.debug(f"Left tactile: {left_tactile}, Right tactile: {right_tactile}")
                    tactile = np.concatenate([left_tactile, right_tactile], axis=0)
                else:
                    tactile = None

                data_dict.add_state(hand_qpos, qpos, np.hstack([ee_pose, head_pose]), tactile=tactile)
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
        if recording is not None and data_dict is not None:
            recording.emergency_save(data_dict, cfg)
        robot.end()

        time.sleep(1.0)
        exit(0)


if __name__ == "__main__":
    main()
