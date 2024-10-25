import concurrent
import logging
from pathlib import Path

from PIL import Image
from tqdm import tqdm

logger = logging.getLogger(__name__)


def save_image(img, key, frame_index, videos_dir: str):
    img = Image.fromarray(img)
    path = Path(videos_dir) / f"{key}_frame_{frame_index:09d}.png"
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(str(path), quality=100)


def save_images_threaded(queue, num_threads=4):
    with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
        futures = []
        while True:
            frame_data = queue.get()
            if frame_data is None:
                logger.info("Exiting save_images_threaded")
                break

            img, key, frame_index, videos_dir = frame_data
            future = executor.submit(save_image, img, key, frame_index, videos_dir)
            futures.append(future)

        with tqdm(total=len(futures), desc="Writing images") as progress_bar:
            concurrent.futures.wait(futures)
            progress_bar.update(len(futures))
