from collections.abc import Sequence

from fourier_dhx.sdk.InspireHand import InspireHand


class InspireDexHand:
    def __init__(self, ip: str, port: int = 2333, timeout: float = 0.1):
        """Simple UDP client for Inspire Dex hand control

        Args:
            ip (str): Hand IP address, usually 192.168.137.19 and 192.168.137.39
            port (int, optional): Hand UDP port. Defaults to 2333.
            timeout (float, optional): UDP timeout. Defaults to 0.1.
        """
        self.hand = InspireHand(ip, timeout)

    def init(self):
        pass

    def reset(self):
        self.set_positions([1000, 1000, 1000, 1000, 1000, 1000])

    def set_positions(self, positions: Sequence[int]):
        if isinstance(positions, list) or isinstance(positions, tuple):
            positions = [int(x) for x in positions]
        else:
            positions = positions.astype(int)
        self.hand.set_angle(positions)

    def get_positions(
        self,
    ):
        angles = self.hand.get_angle()
        return angles

    def stop(self):
        pass
