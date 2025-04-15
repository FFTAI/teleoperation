from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

from teleoperation.service.service import BaseInferenceClient


@dataclass
class ModalityConfig:
    """Configuration for a modality."""

    delta_indices: list[int]
    """Delta indices to sample relative to the current index. The returned data will correspond to the original data at a sampled base index + delta indices."""
    modality_keys: list[str]
    """The keys to load for the modality in the dataset."""


class BasePolicy(ABC):
    @abstractmethod
    def get_action(self, observations: dict[str, Any]) -> dict[str, Any]:
        """
        Abstract method to get the action for a given state.

        Args:
            observations: The observations from the environment.

        Returns:
            The action to take in the environment in dictionary format.
        """
        raise NotImplementedError

    @abstractmethod
    def get_modality_config(self) -> dict[str, ModalityConfig]:
        """
        Return the modality config of the policy.
        """
        raise NotImplementedError


class RobotInferenceClient(BaseInferenceClient, BasePolicy):
    """
    Client for communicating with the RealRobotServer
    """

    def get_action(self, observations: dict[str, Any]) -> dict[str, Any]:
        return self.call_endpoint("get_action", observations)

    def get_modality_config(self) -> dict[str, ModalityConfig]:
        return self.call_endpoint("get_modality_config", requires_input=False)
