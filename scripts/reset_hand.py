import time

from fourier_dhx.sdk.DexHand import DexHand

if __name__ == "__main__":
    left_ip = "192.168.137.19"
    right_ip = "192.168.137.39"
    l_hand = DexHand(left_ip)
    r_hand = DexHand(right_ip)

    l_hand.set_pwm([-200] * 6)

    r_hand.set_pwm([-200] * 6)

    time.sleep(5)

    l_hand.set_pwm([0] * 6)

    r_hand.set_pwm([0] * 6)
