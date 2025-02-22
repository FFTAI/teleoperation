import pickle
import time
from pathlib import Path

import h5py
import numpy as np
import torch
import typer
from omegaconf import DictConfig, OmegaConf
from tqdm import tqdm

from teleoperation.player import ReplayRobot
from teleoperation.utils import CONFIG_DIR, PROJECT_ROOT

DATA_DIR = (PROJECT_ROOT.parent / "data/").resolve()
RECORD_DIR = (DATA_DIR / "recordings/").resolve()
LOG_DIR = (DATA_DIR / "logs/").resolve()


def parse_id(base_dir, prefix):
    base_path = Path(base_dir)
    # Ensure the base path exists and is a directory
    if not base_path.exists() or not base_path.is_dir():
        raise ValueError(f"The provided base directory does not exist or is not a directory: \n{base_path}")

    # Loop through all subdirectories of the base path
    for subfolder in base_path.iterdir():
        if subfolder.is_dir() and subfolder.name.startswith(prefix):
            return str(subfolder), subfolder.name

    # If no matching subfolder is found
    return None, None


def get_norm_stats(data_path):
    with open(data_path, "rb") as f:
        norm_stats = pickle.load(f)
    return norm_stats


def load_policy(policy_path, device):
    policy = torch.jit.load(policy_path, map_location=device)
    return policy


def normalize_input(state, left_img, right_img, norm_stats, last_action_data=None):
    # import ipdb; ipdb.set_trace()
    # left_img = cv2.resize(left_img, (308, 224))
    # right_img = cv2.resize(right_img, (308, 224))
    image_data = torch.from_numpy(np.stack([left_img, right_img], axis=0)) / 255.0
    qpos_data = (torch.from_numpy(state) - norm_stats["qpos_mean"]) / norm_stats["qpos_std"]

    image_data = image_data.view((1, 2, 3, 240, 320)).to(device="cuda")
    qpos_data = qpos_data.view((1, 29)).to(device="cuda")

    if last_action_data is not None:
        last_action_data = torch.from_numpy(last_action_data).to(device="cuda").view((1, -1)).to(torch.float)
        qpos_data = torch.cat((qpos_data, last_action_data), dim=1)
    return (qpos_data.to(torch.float), image_data.to(torch.float))


def merge_act(actions_for_curr_step, k=0.01):
    actions_populated = np.all(actions_for_curr_step != 0, axis=1)
    actions_for_curr_step = actions_for_curr_step[actions_populated]

    exp_weights = np.exp(-k * np.arange(actions_for_curr_step.shape[0]))
    exp_weights = (exp_weights / exp_weights.sum()).reshape((-1, 1))
    raw_action = (actions_for_curr_step * exp_weights).sum(axis=0)

    return raw_action


def main(task_id: str = "", exp_id: str = "", policy: str = "", episode: int = 1, sim: bool = True):
    default_config = Path(CONFIG_DIR) / "body" / "gr1.yaml"
    config = DictConfig(OmegaConf.load(default_config))

    task_dir, task_name = parse_id(RECORD_DIR, task_id)
    if task_dir is None or task_name is None:
        raise ValueError(f"Task ID {task_id} not found in {RECORD_DIR}")
    exp_path, _ = parse_id((Path(LOG_DIR) / task_name).resolve(), exp_id)

    if exp_path is None:
        raise ValueError(f"Experiment ID {exp_id} not found in {LOG_DIR / task_name}")

    episode_name = f"processed_episode_{episode}.hdf5"
    episode_path = (Path(task_dir) / "processed" / episode_name).resolve()
    norm_stat_path = Path(exp_path) / "dataset_stats.pkl"
    # policy_path = Path(exp_path) / f"traced_jit_{ckpt}.pt"
    policy_path = Path(policy)

    play_freq = 30
    temporal_agg = True
    action_dim = 29

    chunk_size = 60
    device = "cuda"

    norm_stats = get_norm_stats(norm_stat_path)
    policy = load_policy(policy_path, device)
    policy.cuda()
    policy.eval()

    if sim:
        data = h5py.File(str(episode_path), "r")
        actions = np.array(data["qpos_action"])
        left_imgs = np.array(data["observation.image.left"])
        right_imgs = np.array(data["observation.image.right"])
        states = np.array(data["observation.state"])
        init_action = np.array(data.attrs["init_action"])
        data.close()
        timestamps = states.shape[0]
    else:
        timestamps = 5000

    history_stack = 0
    # if history_stack > 0:
    #     last_action_queue = deque(maxlen=history_stack)
    #     for i in range(history_stack):
    #         last_action_queue.append(actions[0])
    # else:
    last_action_queue = None
    last_action_data = None

    player = ReplayRobot(DictConfig(config), dt=1 / config.frequency, sim=sim)

    input("Press Enter to start...")

    if temporal_agg:
        all_time_actions = np.zeros([timestamps, timestamps + chunk_size, action_dim])
    else:
        num_actions_exe = chunk_size

    try:
        output = None
        act_index = 0
        for t in tqdm(range(timestamps)):
            try:
                start_1 = time.time()
                if history_stack > 0:
                    last_action_data = np.array(last_action_queue)

                if sim:
                    state, left_image, right_image = states[t], left_imgs[t], right_imgs[t]  # type: ignore
                else:
                    state, left_image, right_image = player.observe()
                data = normalize_input(state, left_image, right_image, norm_stats, last_action_data)

                if temporal_agg:
                    output = policy(*data)[0].detach().cpu().numpy()  # (1,chuck_size,action_dim)
                    all_time_actions[[t], t : t + chunk_size] = output
                    act = merge_act(all_time_actions[:, t])
                else:
                    if output is None or act_index == num_actions_exe - 1:
                        print("Inference...")
                        output = policy(*data)[0].detach().cpu().numpy()
                        act_index = 0
                    act = output[act_index]
                    act_index += 1
                # import ipdb; ipdb.set_trace()
                if history_stack > 0:
                    last_action_queue.append(act)
                act = act * norm_stats["action_std"] + norm_stats["action_mean"]

                print(f"Time taken: {time.time() - start_1}")
                print(f"hz: {1 / (time.time() - start_1)}")
                print(act)

                player.step(act, left_image, right_image)

                taken = time.time() - start_1

                if taken < 1 / play_freq:
                    time.sleep(1 / play_freq - taken)
                else:
                    print("Took too long")
            except Exception as e:
                print(e)
                player.end()
                exit(0)
    except KeyboardInterrupt:
        player.end()
        exit(0)


if __name__ == "__main__":
    typer.run(main)
