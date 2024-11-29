import argparse
import os
from pathlib import Path

import h5py
import numpy as np
from teleoperation.utils import encode_video_frames, match_timestamps
from tqdm import tqdm



DISCARD = 10 # number of frames to discard at the beginning and end of the video

parser = argparse.ArgumentParser()
parser.add_argument("--path", "-p", type=str, help="task folder path")
parser.add_argument("--session", "-s", type=str, help="session name")
parser.add_argument("--output", "-o", type=str, help="output folder path")
args = parser.parse_args()


RECORDING_DIR = Path(args.path).resolve()

if args.session == "all":
    SESSIONS = [ep.stem for ep in RECORDING_DIR.glob("*") if ep.is_dir()]
else:
    SESSIONS = [Path(args.session).resolve().stem]

output_name = RECORDING_DIR.parent.stem + "_" + RECORDING_DIR.stem + "_converted"
OUT_DIR = (Path(args.output) / output_name).resolve()

EXCLUDE_IDS = []

camera_keys = ["top"]

print(f"Recording directory: {RECORDING_DIR}")
print(f"Output directory: {OUT_DIR}")

session_ids = []
for s in SESSIONS:
    session_ids.append((s, sorted([ep.stem.split("_")[-1] for ep in (RECORDING_DIR / s).rglob("*.hdf5")])))
    print(f"Session {s} has {len(session_ids[-1][1])} episodes")
os.makedirs(str(OUT_DIR), exist_ok=True)


def load_video_timestamps(episode_dir):
    with open(str(episode_dir / "rgb_timestamp.txt"), "r") as f:
        lines = [line.strip().split(",") for line in f.readlines()]
        timestamps = [float(line[1]) for line in lines]
        ids = [line[0] for line in lines]

    # discard duplicates
    # find all 000000000
    zero_indices = [i for i, id in enumerate(ids) if int(id) == 0]
    if len(zero_indices) > 1:
        timestamps = timestamps[zero_indices[-1]:]
        ids = ids[zero_indices[-1]:]
        print(f"Discarded {len(zero_indices) - 1} duplicate timestamps at the beginning of the video")
        print(f"New video starts from {timestamps[0]}s at frame {ids[0]}")
        return timestamps[DISCARD:-DISCARD], ids[DISCARD:-DISCARD]
    

    return timestamps[DISCARD:-DISCARD], ids[DISCARD:-DISCARD]

def load_data_timestamps(file_path):
    with h5py.File(str(file_path), "r") as f:
        return np.asarray(f["timestamp"])
    
for s, ids in session_ids:
    for id in ids:

        try:
            # load video timestamps from rgb_timestamps.txt
            video_ts, _ = load_video_timestamps(RECORDING_DIR / s / f"episode_{id}" / "top")
            video_ts = video_ts[120:]
            # shift ids by 120 to account for the extra 120 frames at the beginning of the video
 
            # load data timestamps from hdf5 file
            data_ts = load_data_timestamps(RECORDING_DIR / s / f"episode_{id}.hdf5")

            start_ts = max(video_ts[0], data_ts[0])
            end_ts = min(video_ts[-1], data_ts[-1])
            print(f"Episode {id} starts from {start_ts}s to {end_ts}s")

            # discard frames before and after the timestamps
            video_ts = [ts for ts in video_ts if ts >= start_ts and ts <= end_ts]
            # video_ids = [id for id in video_ids if int(id) >= start_ts * 20 and int(id) <= end_ts * 20]
            data_ts = [ts for ts in data_ts if ts >= start_ts and ts <= end_ts]

            # match video and data timestamps
            matched_ts = match_timestamps(data_ts, video_ts)
            print(f"Matched {len(matched_ts)} timestamps for episode {id}")

            
            if len(video_ts) ==0 or len(data_ts) == 0:
                print(len(video_ts), len(data_ts))
                print(f"Skipping episode {id} because video or data timestamps are empty")
                continue
            print(f"Video starts from {video_ts[0]}s to {video_ts[-1]}s")
            print(f"Data starts from {data_ts[0]}s to {data_ts[-1]}s")

            vid_out_dir = OUT_DIR / s / f"episode_{id}.mp4"

            encode_video_frames(RECORDING_DIR / s / f"episode_{id}" / "top", vid_out_dir, 15, overwrite=True, start_frame=120)
        except Exception as e:
            print(f"Error processing episode {id}: {e}")
            continue


# out_id = 0
# for s, ids in session_ids:
#     for id in ids:
#         # load video timestamps from rgb_timestamps.txt
#         with open(str(RECORDING_DIR / s / f"episode_{id}/rgb_timestamps.txt"), "r") as f:
#             image_timestamps = [float(line.strip()) for line in f.readlines()]

#         out_id += 1


        
# print("Encoding videos...")
# image_timestamps = {}
# for s, ids in session_ids:
#     for id in tqdm(ids):
#         id_ind = int(id)
#         if id_ind % 2 == 1:
#             continue
#         output_id = id_ind // 2

#         vid_out_dir = OUT_DIR / f"{output_id:09d}"
#         encode_video_frames(RECORDING_DIR / s / f"episode_{id}", vid_out_dir / "ego_view.mp4", 20, overwrite=True)

#         image_timestamps[id] = [
#             f * 1 / 20 for f, _ in enumerate((RECORDING_DIR / s / f"episode_{id}").rglob("rgb_frame_*.png"))
#         ]


# with h5py.File(str(OUT_DIR / "trainable_data.hdf5"), "w", rdcc_nbytes=1024**2 * 2) as f:
#     data = f.create_group("data")

#     metadata = f.create_group("metadata")

#     metadata_dict = {
#         "trajectory_durations": [],
#         "trajectory_ids": [],
#         "trajectory_lengths": [],
#         "whitelist": [],
#     }

#     print("Converting hdf5 files...")
#     for s, ids in session_ids:
#         for id in tqdm(ids):
#             id_ind = int(id)
#             if id_ind % 2 == 1:
#                 continue
#             output_id = id_ind // 2
#             episode_data = data.create_group(f"{output_id:09d}")
#             with h5py.File(RECORDING_DIR / s / f"episode_{id}.hdf5", "r") as f:
#                 data_ts = np.asarray(f["timestamp"])
#                 non_duplicate = np.where(np.diff(data_ts) > 0)[0]
#                 image_ts = np.asarray([ts + data_ts[0] for ts in image_timestamps[id]])
#                 image_ts = filter(lambda x: x < data_ts[-1], image_ts)
#                 matched = match_timestamps(data_ts[non_duplicate], image_ts)
#                 matched_ts = data_ts[matched]

#                 metadata_dict["trajectory_durations"].append(matched_ts[-1] - matched_ts[0])
#                 metadata_dict["trajectory_ids"].append(f"{output_id:09d}")
#                 metadata_dict["trajectory_lengths"].append(len(matched_ts))
#                 if int(id) in EXCLUDE_IDS:
#                     metadata_dict["whitelist"].append(False)
#                 else:
#                     metadata_dict["whitelist"].append(True)

#                 action = episode_data.create_group("action")
#                 state = episode_data.create_group("state")
#                 for k in ["hand", "pose", "robot"]:
#                     action.create_dataset(k, data=f[f"action/{k}"][matched])
#                     state.create_dataset(k, data=f[f"state/{k}"][matched])
#                 episode_data.create_dataset("timestamp", data=f["timestamp"][matched])

#     metadata.create_dataset("trajectory_durations", data=metadata_dict["trajectory_durations"])
#     metadata.create_dataset("trajectory_ids", data=metadata_dict["trajectory_ids"])
#     metadata.create_dataset("trajectory_lengths", data=metadata_dict["trajectory_lengths"])
#     metadata.create_dataset("whitelist", data=metadata_dict["whitelist"])
