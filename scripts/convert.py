import os
import sys
import h5py
from pathlib import Path

import numpy as np
from silverscreen.utils import encode_video_frames, match_timestamps
from silverscreen.data_collection import EpisodeDataDict
from tqdm import tqdm

RECORDING_DIR = (Path(__file__).parent.parent / Path("data/recordings/nv-cola/")).resolve()
SESSIONS = ["2024-10-27_12-19-06"]

OUT_DIR = (Path(__file__).parent.parent / Path("data/exports/nv-cola/setup-1")).resolve()

print(RECORDING_DIR)
print(OUT_DIR)

session_ids = []
for s in SESSIONS:
    session_ids.append((s, sorted([ep.stem.split("_")[-1] for ep in (RECORDING_DIR / s).rglob("*.hdf5")])))

os.makedirs(str(OUT_DIR), exist_ok=True)

print("Encoding videos...")
image_timestamps = {}
for s, ids in session_ids:
    for id in tqdm(ids):
        id_ind = int(id)
        if id_ind % 2 == 1:
            continue
        output_id = id_ind // 2

        vid_out_dir = OUT_DIR / f"{output_id:09d}"
        encode_video_frames(RECORDING_DIR / s / f"episode_{id}", vid_out_dir / "ego_view.mp4", 20, overwrite=True)

        image_timestamps[id] = [
            f * 1 / 20 for f, _ in enumerate((RECORDING_DIR / s / f"episode_{id}").rglob("rgb_frame_*.png"))
        ]


with h5py.File(str(OUT_DIR / "trainable_data.hdf5"), "w", rdcc_nbytes=1024**2 * 2) as f:
    data = f.create_group("data")

    metadata = f.create_group("metadata")

    metadata_dict = {
        "trajectory_durations": [],
        "trajectory_ids": [],
        "trajectory_lengths": [],
        "whitelist": [],
    }

    print("Converting hdf5 files...")
    for s, ids in session_ids:
        for id in tqdm(ids):
            id_ind = int(id)
            if id_ind % 2 == 1:
                continue
            output_id = id_ind // 2
            episode_data = data.create_group(f"{output_id:09d}")
            with h5py.File(RECORDING_DIR / s / f"episode_{id}.hdf5", "r") as f:
                data_ts = np.asarray(f["timestamp"])
                image_ts = np.asarray([ts + data_ts[0] for ts in image_timestamps[id]])
                matched = match_timestamps(data_ts, image_ts)
                matched_ts = data_ts[matched]
                metadata_dict["trajectory_durations"].append(matched_ts[-1] - matched_ts[0])
                metadata_dict["trajectory_ids"].append(f.attrs["episode_id"])
                metadata_dict["trajectory_lengths"].append(len(matched_ts))
                metadata_dict["whitelist"].append(True)

                action = episode_data.create_group("action")
                state = episode_data.create_group("state")
                for k in ["hand", "pose", "robot"]:
                    action.create_dataset(k, data=f[f"action/{k}"][matched])
                    state.create_dataset(k, data=f[f"state/{k}"][matched])

                episode_data.create_dataset("timestamp", data=f["timestamp"][matched])

    metadata.create_dataset("trajectory_durations", data=metadata_dict["trajectory_durations"])
    metadata.create_dataset("trajectory_ids", data=metadata_dict["trajectory_ids"])
    metadata.create_dataset("trajectory_lengths", data=metadata_dict["trajectory_lengths"])
    metadata.create_dataset("whitelist", data=metadata_dict["whitelist"])
