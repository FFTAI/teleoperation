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

    def next(self):
        if self.state == FSM.State.INITIALIZED:
            self.state = FSM.State.STARTED
        elif self.state == FSM.State.STARTED:
            self.state = FSM.State.CALIBRATING
        elif self.state == FSM.State.CALIBRATING:
            self.state = FSM.State.CALIBRATED
        elif self.state == FSM.State.CALIBRATED:
            self.state = FSM.State.ENGAGED
        elif self.state == FSM.State.ENGAGED:
            self.state = FSM.State.EPISODE_STARTED
        elif self.state == FSM.State.EPISODE_STARTED:
            self.state = FSM.State.COLLECTING
        elif self.state == FSM.State.COLLECTING:
            self.state = FSM.State.EPISODE_ENDED
        elif self.state == FSM.State.EPISODE_ENDED:
            self.state = FSM.State.IDLE
        elif self.state == FSM.State.IDLE:
            self.state = FSM.State.EPISODE_STARTED

        logger.info(f"[FSM] State transition: {self.prev_state} -> {self.state}")

    def disenage(self):
        self.state = FSM.State.CALIBRATED
