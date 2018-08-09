"""
Created by Alejandro Daniel Noel
"""

import sounddevice as sd
import numpy as np
from time import sleep


sampling_freq = 48000


class Wave:
    def __init__(self, wave=None, channel=1):
        self.wave = wave if wave is not None else np.ones(sampling_freq)
        self._delay = 0.0
        self._shift = 0.0
        self._attack = 0.0
        self._sustain = 0.0
        self._release = 0.0
        self.channel = channel

    def __add__(self, other):
        if other.channel != self.channel:
            if self.channel == 1:
                return Wave(np.column_stack((self.wave, other.wave)))
            else:
                return Wave(np.column_stack((other.wave, self.wave)))
        else:
            return Wave(self.wave + other.wave)

    def __mul__(self, other):
        if isinstance(other, Wave):
            return Wave(self.wave * other.wave)
        else:
            return Wave(self.wave * other)

    @property
    def delay(self):
        return self._delay

    @delay.setter
    def delay(self, value):
        self._delay = value
        pos = int(value * sampling_freq)
        self.wave = np.roll(self.wave, pos)
        self.wave[:pos] *= 0.0

    @property
    def shift(self):
        return self._shift

    @shift.setter
    def shift(self, value):
        self._delay = value
        pos = int(value * sampling_freq)
        self.wave = np.roll(self.wave, pos)

    @property
    def attack(self):
        return self._attack

    @attack.setter
    def attack(self, value):
        self._attack = value
        points = int(value * sampling_freq)
        start = int(self.delay * sampling_freq)
        self.wave[start:start + points] *= np.linspace(0.0, 1.0, points)

    @property
    def sustain(self):
        return self._sustain

    @sustain.setter
    def sustain(self, value):
        self._sustain = value

    @property
    def release(self):
        return self._release

    @release.setter
    def release(self, value):
        self._release=value
        points= int(value * sampling_freq)
        start= int((self.sustain + self.attack +self.delay) * sampling_freq)
        self.wave[start:start + points] *= np.linspace(1.0, 0.0, points)
        self.wave[start + points:] *= 0.0

    def play(self, duration, blocking=False):
        if self.wave.ndim == 1:
            sd.play(self.wave, loop=True, mapping=self.channel)
        else:
            sd.play(self.wave, loop=True, mapping=(1, 2))
        if blocking:
            sleep(duration)
            sd.stop()

    def stop(self):
        sd.stop()


class Sine(Wave):
    def __init__(self, frequency, amplitude=1.0, **kwargs):
        self.frequency = frequency
        self.amplitude = amplitude
        Wave.__init__(self, np.sin(np.linspace(0.0, 2 * np.pi * frequency, sampling_freq)), **kwargs)


if __name__ == "__main__":
    awave1 = Sine(frequency=942)
    awave1.delay = 0.231
    awave1.attack = 0.071
    awave1.sustain = 0.479
    awave1.release = 0.043
    awave1.channel = 1

    awave2 = Sine(frequency=825)
    awave2.delay = 0.0
    awave2.attack = 0.059
    awave2.sustain = 0.465
    awave2.release = 0.040
    awave2.channel = 2

    awave3 = awave1 + awave2
    awave3 *= 0.5
    awave3.play(1.0,device=0, blocking=True)