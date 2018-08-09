"""
Created by Ale and Nat
"""

from random import randint
from audio_lib import Sine


sampling_freq = 48000


class Alegorithm:
    def __init__(self):
        pass

    def play_background_turbulence(self, speed, distance=0.1):
        turbulence1 = Sine(frequency=randint(35, 55)) + Sine(frequency=randint(35, 55))
        turbulence2 = Sine(frequency=randint(35, 55)) + Sine(frequency=randint(35, 55))

        pulse1 = Sine(frequency=80)
        pulse1.attack = 0.071
        pulse1.sustain = 0.479
        pulse1.release = 0.043
        pulse1.channel = 1

        pulse2 = Sine(frequency=80)
        pulse2.delay = distance / speed
        pulse2.attack = 0.071
        pulse2.sustain = 0.479
        pulse2.release = 0.043
        pulse2.channel = 2

        combined_signal = turbulence1 * 0.3 + turbulence2 * 0.3 + pulse1 + pulse2

        combined_signal.play(1.0, blocking=True)


if __name__ == "__main__":
    test = Alegorithm()
    test.play_background_turbulence(0.5)


