import concurrent
import logging
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from tqdm import tqdm

logger = logging.getLogger(__name__)


def save_image(img, key, frame_index, videos_dir: str):
    img = Image.fromarray(img)
    path = Path(videos_dir) / f"{key}_frame_{frame_index:09d}.png"
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(str(path), quality=100)


def save_timestamp(timestamp, key, frame_index, videos_dir: str):
    path = Path(videos_dir) / f"{key}_timestamp.txt"
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a") as f:
        # append timestamp to file
        f.write(f"{frame_index:09d},{timestamp}\n")


def save_images_threaded(queue, num_threads=4):
    with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
        futures = []
        while True:
            frame_data = queue.get()
            if frame_data is None:
                logger.info("Exiting save_images_threaded")
                break

            img, key, frame_index, videos_dir, timestamp = frame_data
            future = executor.submit(save_image, img, key, frame_index, videos_dir)
            save_timestamp_future = executor.submit(save_timestamp, timestamp, key, frame_index, videos_dir)
            futures.append(future)
            futures.append(save_timestamp_future)

        with tqdm(total=len(futures), desc="Writing images") as progress_bar:
            concurrent.futures.wait(futures)
            progress_bar.update(len(futures))


def post_process(
    data_dict: dict[str, np.ndarray], shape: tuple[int, int], crop_sizes: tuple[int, int, int, int]
) -> dict[str, np.ndarray]:
    for source, data in data_dict.items():
        data_dict[source] = _post_process(source, data, shape, crop_sizes)
    return data_dict


def _post_process(
    source: str, data: np.ndarray, shape: tuple[int, int], crop_sizes: tuple[int, int, int, int]
) -> np.ndarray:
    # cropped_img_shape = (240, 320) hxw
    # crop_sizes = (0, 0, int((1280-960)/2), int((1280-960)/2)) # (h_top, h_bottom, w_left, w_right)
    shape = (shape[1], shape[0])  # (w, h)
    crop_h_top, crop_h_bottom, crop_w_left, crop_w_right = crop_sizes
    if source == "left" or source == "depth":
        data = data[crop_h_top:-crop_h_bottom, crop_w_left:-crop_w_right]
        data = cv2.resize(data, shape)
    elif source == "right":
        data = data[crop_h_top:-crop_h_bottom, crop_w_right:-crop_w_left]
        data = cv2.resize(data, shape)

    return data
