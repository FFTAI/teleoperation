import argparse
import concurrent.futures
import os
import time
from pathlib import Path

import cv2
import h5py
import numpy as np
import pyzed.sl as sl
from numpy.lib.stride_tricks import as_strided

EXPORT_FOLDER_NAME = "processed"


def load_svo(
    path,
):
    input_file = path + ".svo2"
    # import ipdb; ipdb.set_trace()
    print(input_file)

    crop_size_h = 0
    crop_size_w = 1280 - 960
    init_parameters = sl.InitParameters()
    init_parameters.set_from_svo_file(input_file)

    zed = sl.Camera()
    err = zed.open(init_parameters)
    left_image = sl.Mat()
    right_image = sl.Mat()

    nb_frames = zed.get_svo_number_of_frames()
    print("Total image frames: ", nb_frames)

    cropped_img_shape = (240, 320)
    left_imgs = np.zeros((nb_frames, 3, cropped_img_shape[0], cropped_img_shape[1]), dtype=np.uint8)
    right_imgs = np.zeros((nb_frames, 3, cropped_img_shape[0], cropped_img_shape[1]), dtype=np.uint8)
    timestamps = np.zeros((nb_frames,), dtype=np.int64)
    cnt = 0
    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(left_image, sl.VIEW.LEFT)
            zed.retrieve_image(right_image, sl.VIEW.RIGHT)

            timestamps[cnt] = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds()
            # import ipdb; ipdb.set_trace()
            # resize to 640x480
            left_img = cv2.cvtColor(
                left_image.get_data()[crop_size_h:, crop_size_w:],
                cv2.COLOR_BGRA2RGB,
            )
            left_imgs[cnt] = cv2.resize(left_img, (cropped_img_shape[1], cropped_img_shape[0])).transpose(2, 0, 1)
            right_img = cv2.cvtColor(
                right_image.get_data()[crop_size_h:, 0:-crop_size_w],
                cv2.COLOR_BGRA2RGB,
            )
            right_imgs[cnt] = cv2.resize(right_img, (cropped_img_shape[1], cropped_img_shape[0])).transpose(2, 0, 1)
            cnt += 1
            if cnt % 100 == 0:
                print(f"{cnt/nb_frames*100:.2f}%")
                # plt.imsave(f"left_img_{cnt}.png", left_imgs[cnt-1].transpose(1, 2, 0))
        elif zed.grab() == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            break
    # print delta mean and std for img_timstamps
    delta = np.diff(timestamps)[:-1]
    print("img timestamps delta mean: ", np.mean(delta))
    print("img timestamps delta std: ", np.std(delta))
    return left_imgs[10:-10], right_imgs[10:-10], timestamps[10:-10]


def load_svo_old(path, crop_size_h=240, crop_size_w=320):
    input_file = path + ".svo2"
    # import ipdb; ipdb.set_trace()
    print(input_file)
    crop_size_h = crop_size_h
    crop_size_w = crop_size_w
    init_parameters = sl.InitParameters()
    init_parameters.set_from_svo_file(input_file)

    zed = sl.Camera()
    err = zed.open(init_parameters)
    left_image = sl.Mat()
    right_image = sl.Mat()

    nb_frames = zed.get_svo_number_of_frames()
    print("Total image frames: ", nb_frames)

    cropped_img_shape = (720 - crop_size_h, 1280 - 2 * crop_size_w)
    left_imgs = np.zeros((nb_frames, 3, cropped_img_shape[0], cropped_img_shape[1]), dtype=np.uint8)
    right_imgs = np.zeros((nb_frames, 3, cropped_img_shape[0], cropped_img_shape[1]), dtype=np.uint8)
    timestamps = np.zeros((nb_frames,), dtype=np.int64)
    cnt = 0
    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(left_image, sl.VIEW.LEFT)
            zed.retrieve_image(right_image, sl.VIEW.RIGHT)

            timestamps[cnt] = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds()
            # import ipdb; ipdb.set_trace()
            left_imgs[cnt] = cv2.cvtColor(
                left_image.get_data()[crop_size_h:, crop_size_w:-crop_size_w],
                cv2.COLOR_BGRA2RGB,
            ).transpose(2, 0, 1)
            right_imgs[cnt] = cv2.cvtColor(
                right_image.get_data()[crop_size_h:, crop_size_w:-crop_size_w],
                cv2.COLOR_BGRA2RGB,
            ).transpose(2, 0, 1)
            cnt += 1
            if cnt % 100 == 0:
                print(f"{cnt/nb_frames*100:.2f}%")
                # plt.imsave(f"left_img_{cnt}.png", left_imgs[cnt-1].transpose(1, 2, 0))
        elif zed.grab() == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            break
    # print delta mean and std for img_timstamps
    delta = np.diff(timestamps)[:-1]
    print("img timestamps delta mean: ", np.mean(delta))
    print("img timestamps delta std: ", np.std(delta))
    return left_imgs[10:-10], right_imgs[10:-10], timestamps[10:-10]


def load_hdf5(path, offset=10):  # offset 10ms
    input_file = path + ".hdf5"
    file = h5py.File(input_file, "r")
    print(f"Total hdf5_frames: {file['/timestamp'].shape[0]}")
    # print(file["/obs/timestamp"].shape)
    # print(file["/obs/qpos"].shape)
    # print(file["/obs/qvel"].shape)
    # print(file["/action/joint_pos"].shape)
    # print("keys: ", list(file.keys()))
    timestamps = np.array(file["/timestamp"][:], dtype=np.int64) - offset
    states_joints = np.array(file["/obs/qpos"][:])
    states_hands = np.array(file["/obs/hand_qpos"][:])
    actions_joints = np.array(file["/action/joints"][:])
    actions_hands = np.array(file["/action/hands"][:])
    # cmds = np.array(file["/action/cmd"][:])

    states = np.concatenate([states_joints[:, [13, 16, 17]], states_joints[:, -14:], states_hands], axis=1)
    actions = np.concatenate(
        [actions_joints[:, [13, 16, 17]], actions_joints[:, -14:], actions_hands],
        axis=1,
    )
    cmds = []
    return timestamps, states, actions, cmds


def match_timestamps(candidate, ref):
    closest_indices = []
    # candidate = np.sort(candidate)
    for t in ref:
        idx = np.searchsorted(candidate, t, side="left")
        if idx > 0 and (idx == len(candidate) or np.fabs(t - candidate[idx - 1]) < np.fabs(t - candidate[idx])):
            closest_indices.append(idx - 1)
        else:
            closest_indices.append(idx)
    # print("closest_indices: ", len(closest_indices))
    return np.array(closest_indices)


def find_all_episodes(path):
    episodes = [os.path.join(path, f) for f in os.listdir(path) if f.startswith("episode") and f.endswith(".svo2")]
    episodes = [os.path.basename(ep).split(".")[0] for ep in episodes]
    return episodes


def create_chunks(data, chunk_size):
    N, F = data.shape
    if chunk_size > N:
        raise ValueError("chunk_size cannot be greater than N.")

    stride0, stride1 = data.strides
    new_shape = (N - chunk_size + 1, chunk_size, F)
    new_strides = (stride0, stride0, stride1)

    return as_strided(data, shape=new_shape, strides=new_strides)


def process_episode(file_name, ep):
    left_imgs, right_imgs, img_timestamps = load_svo(file_name)
    hdf5_timestamps, states, actions, _ = load_hdf5(file_name)
    closest_indices = match_timestamps(candidate=hdf5_timestamps, ref=img_timestamps)

    timesteps = len(closest_indices)
    qpos_actions = actions[closest_indices]
    # cmds = cmds[closest_indices]

    # save_video(left_imgs, file_name + ".mp4")
    path = os.path.dirname(file_name)
    all_data_path = os.path.join(path, EXPORT_FOLDER_NAME)
    os.makedirs(all_data_path, exist_ok=True)

    with h5py.File(all_data_path + f"/processed_{ep}.hdf5", "w") as hf:
        start = time.time()
        hf.create_dataset("observation.image.left", data=left_imgs)
        hf.create_dataset("observation.image.right", data=right_imgs)
        # hf.create_dataset("cmds", data=cmds.astype(np.float32))
        hf.create_dataset("observation.state", data=states[closest_indices].astype(np.float32))
        hf.create_dataset("qpos_action", data=qpos_actions.astype(np.float32))
        hf.attrs["sim"] = False
        hf.attrs["compress"] = True
        hf.attrs["init_action"] = qpos_actions[0].astype(np.float32)

        print("Time to save dataset: ", time.time() - start)


def process_all_episodes(all_eps, path):
    results = []
    with concurrent.futures.ProcessPoolExecutor() as executor:
        future_to_ep = {executor.submit(process_episode, os.path.join(path, ep), ep): ep for ep in all_eps}
        for future in concurrent.futures.as_completed(future_to_ep):
            ep = future_to_ep[future]
            try:
                result = future.result()
                results.append(result)
            except Exception as e:
                print(f"Episode {ep} generated an exception: {e}")
    return results


def save_video(left_imgs, path):
    _, height, width = left_imgs[0].shape
    print(f"width: {width}, height: {height}")
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_writer = cv2.VideoWriter(path, fourcc, 60, (width, height))

    for img in left_imgs:
        # print(img.shape)
        img_bgr = cv2.cvtColor(img.transpose(1, 2, 0), cv2.COLOR_RGB2BGR)
        video_writer.write(img_bgr)

    video_writer.release()


def find_all_processed_episodes(path):
    episodes = [f for f in os.listdir(path)]
    return episodes


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--name", type=str, required=True)
    parser.add_argument("-s", "--save_video", action="store_true", default=False)

    args = parser.parse_args()

    root = "../data/recordings"
    folder_name = args.name

    path = os.path.join(root, folder_name)

    all_eps = find_all_episodes(path)
    print(all_eps)

    if args.save_video:
        file_name = path + "/" + all_eps[0]
        print("saving video for file: ", file_name)
        left_imgs, right_imgs, img_timestamps = load_svo(file_name)
        os.makedirs(os.path.join(path, "videos"), exist_ok=True)
        save_video(left_imgs, os.path.join(path, "videos", "sample.mp4"))
    else:
        for ep in all_eps:
            file_name = path + "/" + ep
            process_episode(file_name, ep)
            print("processed file", file_name)

    # print len
    folder_path = Path(root) / folder_name / EXPORT_FOLDER_NAME

    episodes = find_all_processed_episodes(folder_path)
    num_episodes = len(episodes)
    lens = []

    for episode in episodes:
        episode_path = folder_path / episode

        data = h5py.File(str(episode_path), "r")
        lens.append(data["qpos_action"].shape[0])
        data.close()

    lens = np.array(lens)
    episodes = np.array(episodes)
    print(lens[np.argsort(lens)])
    print(episodes[np.argsort(lens)])
    # results = process_all_episodes(all_eps, path)
    # print(len(results))
