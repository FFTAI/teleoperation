import concurrent
import logging
import multiprocessing as mp
import threading
import time
from multiprocessing import shared_memory
from pathlib import Path
from typing import Literal

import cv2
import numpy as np
from tqdm import tqdm

from teleoperation.utils import posix_to_iso

logger = logging.getLogger(__name__)
cv2.setNumThreads(1)


class RecordCamera:
    def __init__(self, num_processes: int = 1, num_threads: int = 4, queue_size: int = 30):
        super().__init__()
        self.num_processes = num_processes
        self.num_threads = num_threads
        self.save_queue = mp.Queue(maxsize=queue_size)
        self.processes = []
        self.delete_event = mp.Event()

    # def delete(self, video_path: str):
    #     self.delete_event.set()
    #     # empty the queue
    #     while not self.save_queue.empty():
    #         self.save_queue.get()
    #     delete_dir(video_path)
    #     self.delete_event.clear()

    def put(self, frames: dict[str, np.ndarray | None], frame_id: int, video_path: str, timestamp: float):
        for key, frame in frames.items():
            if frame is not None:
                self.save_queue.put((frame, key, frame_id, video_path, timestamp))

    def start(self):
        for _ in range(self.num_processes):
            p = mp.Process(
                target=save_images_threaded, args=(self.save_queue, self.num_threads, self.delete_event), daemon=True
            )
            p.start()
            self.processes.append(p)

    def stop(self):
        if self.save_queue is not None:
            self.save_queue.put(None)

        for p in self.processes:
            p.join()


class DisplayCamera:
    def __init__(
        self,
        mode: Literal["mono", "stereo"],
        resolution: tuple[int, int],
        crop_sizes: tuple[int, int, int, int],
    ):
        self.mode = mode
        self.crop_sizes = [s if s != 0 else None for s in crop_sizes]

        t, b, l, r = crop_sizes
        resolution_cropped = (
            resolution[0] - t - b,
            resolution[1] - l - r,
        )

        self.shape = resolution_cropped

        num_images = 2 if mode == "stereo" else 1

        display_img_shape = (resolution_cropped[0], num_images * resolution_cropped[1], 3)
        self.shm = shared_memory.SharedMemory(
            create=True,
            size=np.prod(display_img_shape) * np.uint8().itemsize,  # type: ignore
        )
        self.image_array = np.ndarray(
            shape=display_img_shape,
            dtype=np.uint8,
            buffer=self.shm.buf,
        )
        self.lock = threading.Lock()

        self._video_path = mp.Array("c", bytes(256))
        self._flag_marker = False

    @property
    def shm_name(self) -> str:
        return self.shm.name

    @property
    def shm_size(self) -> int:
        return self.shm.size

    def put(self, data: dict[str, np.ndarray], marker=False):
        t, b, l, r = self.crop_sizes

        if self.mode == "mono":
            if "rgb" in data:
                display_img = data["rgb"][t : None if b is None else -b, l : None if r is None else -r]
            elif "left" in data:
                display_img = data["left"][t : None if b is None else -b, l : None if r is None else -r]
            else:
                raise ValueError("Invalid data.")
        elif self.mode == "stereo":
            display_img = np.hstack(
                (
                    data["left"][t : None if b is None else -b, l : None if r is None else -r],
                    data["right"][t : None if b is None else -b, r : None if l is None else -l],
                )
            )
        else:
            raise ValueError("Invalid mode.")

        if marker:
            # draw markers on left and right frames
            width = display_img.shape[1]
            hieght = display_img.shape[0]

            if self.mode == "mono":
                display_img = cv2.circle(display_img, (int(width // 2), int(hieght * 0.2)), 15, (255, 0, 0), -1)
            elif self.mode == "stereo":
                display_img = cv2.circle(display_img, (int(width // 2 * 0.5), int(hieght * 0.2)), 15, (255, 0, 0), -1)
                display_img = cv2.circle(display_img, (int(width // 2 * 1.5), int(hieght * 0.2)), 15, (255, 0, 0), -1)
        with self.lock:
            np.copyto(self.image_array, display_img)


def save_image(img, key, frame_index, videos_dir: str, pixel_mode: str = "RGB", extension: str = "png"):
    if pixel_mode == "RGB":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif pixel_mode == "BGR":
        pass
    else:
        raise ValueError("Invalid pixel_mode.")
    path = Path(videos_dir) / f"{key}_frame_{frame_index:09d}.{extension}"
    path.parent.mkdir(parents=True, exist_ok=True)

    cv2.imwrite(str(path), img)


def save_image_by_ts(img, key, timestamp, videos_dir: str, pixel_mode: str = "RGB", extension: str = "png"):
    if "depth" in key or len(img.shape) == 2 or img.dtype == np.uint16:
        pixel_mode = "depth"
    if pixel_mode == "RGB":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif pixel_mode == "BGR" or pixel_mode == "depth":
        pass
    else:
        raise ValueError("Invalid pixel_mode.")
    iso_ts = posix_to_iso(timestamp)
    path = Path(videos_dir) / f"{key}" / f"{iso_ts}.{extension}"
    path.parent.mkdir(parents=True, exist_ok=True)

    cv2.imwrite(str(path), img)

    # img = Image.fromarray(img)
    # img.save(str(path), quality=100)


def delete_dir(videos_dir: str):
    path = Path(videos_dir).resolve()
    while True:
        for file in path.glob("*"):
            if file.is_file():
                file.unlink()
            else:
                delete_dir(str(file))
        try:
            path.rmdir()
            logger.info(f"Deleted directory: {path}")
            return
        except:
            logger.info(f"Failed to delete directory: {path}")
            time.sleep(1 / 60)


def delete_if_exists(dir: str):
    path = Path(dir)
    if path.exists():
        delete_dir(dir)


def save_timestamp(timestamp, key, frame_index, videos_dir: str):
    path = Path(videos_dir) / f"{key}_timestamp.txt"
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a") as f:
        # append timestamp to file
        f.write(f"{frame_index:09d},{timestamp}\n")


def save_images_threaded(queue, num_threads=4, deletion_event=None):
    with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
        futures = []
        while True:
            if deletion_event is not None and deletion_event.is_set():
                time.sleep(1 / 60)
                continue
            frame_data = queue.get()
            if frame_data is None:
                logger.info("Exiting save_images_threaded")
                break

            img, key, frame_index, videos_dir, timestamp = frame_data
            future = executor.submit(save_image_by_ts, img, key, timestamp, videos_dir)
            # save_timestamp_future = executor.submit(save_timestamp, timestamp, key, frame_index, videos_dir)
            futures.append(future)
            # futures.append(save_timestamp_future)

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
