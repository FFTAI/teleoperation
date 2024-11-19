import logging
from enum import Enum

logger = logging.getLogger(__name__)


class FSM:
    class State(Enum):
        INITIALIZED = "INITIALIZED"
        STARTED = "STARTED"
        CALIBRATING = "CALIBRATING"
        CALIBRATED = "CALIBRATED"
        ENGAGED = "ENGAGED"
        IDLE = "IDLE"
        EPISODE_STARTED = "EPISODE_STARTED"
        COLLECTING = "COLLECTING"
        EPISODE_ENDED = "EPISODE_ENDED"
        EPISODE_DISCARDED = "EPISODE_DISCARDED"

    def __init__(self) -> None:
        self.prev_state = FSM.State.INITIALIZED
        self._state = FSM.State.INITIALIZED

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self.prev_state = self._state
        self._state = value
        logger.info(f"[FSM] transition {self.prev_state} -> {self.state}")

    def disenage(self, robot):
        robot.pause_robot()
        self.state = FSM.State.IDLE
