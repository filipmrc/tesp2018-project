"""
Created by Alejandro Daniel Noel
"""

from audio_lib import Sine


def on_signal():
    pulse1 = Sine(frequency=80)
    pulse1.attack = 0.071
    pulse1.sustain = 0.479
    pulse1.release = 0.043
    pulse1.channel = 1
    pulse1.play(1.0, blocking=True)
