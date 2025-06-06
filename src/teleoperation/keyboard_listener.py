import asyncio
import threading
import time
from functools import cached_property

import evdev
from evdev import InputDevice, ecodes
from loguru import logger


class KeyboardListener:
    def __init__(self, device_path: str):
        self.device = InputDevice(device_path)

        self._pressed_keys = set()
        self.running = True

    async def listen(self):
        logger.info(f"Listening for keyboard events on {self.device.path}")
        async for event in self.device.async_read_loop():
            if not self.running:
                break

            if event.type == 1:  # Key event
                if event.value == 1:  # Key down
                    self._pressed_keys.add(event.code)
                    # print(f"Key pressed: {ecodes.KEY[event.code]}, {event.code == ecodes.KEY_SPACE}")
                elif event.value == 0:  # Key up
                    self._pressed_keys.discard(event.code)
                    # print(f"Key released: {event.code}")
                elif event.value == 2:  # key hold
                    pass

    @property
    def pressed_keys(self):
        return self._pressed_keys

    @cached_property
    def _key_to_code(self):
        return {v: k for k, v in ecodes.KEY.items()}

    def is_pressed(self, key: str):
        key = f"KEY_{key.upper()}"
        code = self._key_to_code[key]
        if code in self._pressed_keys:
            logger.info(f"Key pressed: {key}, {code}")
            self._pressed_keys.discard(code)
            return True
        return False

    def stop(self):
        self.running = False
        self.device.close()
        print("Stopped listening to keyboard events.")


def start_keyboard_listener(device_path: str):
    listener = KeyboardListener(device_path)
    _ = threading.Thread(target=asyncio.run, args=(listener.listen(),), daemon=True).start()
    return listener


def detect_keyboard():
    for dev_path in evdev.list_devices():
        device = InputDevice(dev_path)

        if "keyboard" not in device.name.lower():
            continue

        key_capabilities = device.capabilities().get(ecodes.EV_KEY, None)
        if key_capabilities is None:
            continue

        # Check if the keyboard has all the required keys
        if (
            ecodes.KEY_SPACE in key_capabilities
            and ecodes.KEY_Q in key_capabilities
            and ecodes.KEY_X in key_capabilities
            and ecodes.KEY_D in key_capabilities
            and ecodes.KEY_S in key_capabilities
            and ecodes.KEY_Z in key_capabilities
            and ecodes.KEY_P in key_capabilities
        ):
            return dev_path
    return None


# try:
#     from pynput import keyboard
# except ImportError:
#     logger.warning("pynput import failed. KeyboardListener will not work.")

# class KeyboardListener:
#     def __init__(self):
#         self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
#         self._key_pressed = defaultdict(bool)
#         logger.debug("Keyboard listener initialized")

#     @property
#     def key_pressed(self):
#         out_key_pressed = deepcopy(self._key_pressed)
#         self._key_pressed = defaultdict(bool)
#         return out_key_pressed

#     @property
#     def space_pressed(self):
#         return self._key_pressed.get("space", False)

#     def start(self):
#         self.listener.start()

#     def on_press(self, key):
#         try:
#             if isinstance(key, keyboard.KeyCode):
#                 self._key_pressed[key.char] = True
#             elif isinstance(key, keyboard.Key):
#                 self._key_pressed[key.name] = True

#         except AttributeError:
#             pass

#     def on_release(self, key):
#         try:
#             if key == keyboard.Key.space:
#                 self._space_pressed = False
#         except AttributeError:
#             pass

#     def stop(self):
#         self.listener.stop()

if __name__ == "__main__":
    import asyncio
    import sys

    if len(sys.argv) != 2:
        print("Usage: python keyboard_listener.py <device_path>")
        sys.exit(1)

    device_path = sys.argv[1]
    listener = KeyboardListener(device_path)

    listener_thread = threading.Thread(target=asyncio.run, args=(listener.listen(),), daemon=True).start()

    try:
        while True:
            if listener.is_pressed("space"):
                print("Space key was pressed!")
            time.sleep(0.05)
    except KeyboardInterrupt:
        listener.stop()
        print("Exiting keyboard listener.")
