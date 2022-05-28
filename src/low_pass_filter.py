import numpy as np
import random
from matplotlib import pyplot as plt


class LowPassFilter:
    def __init__(self, cutoff_freq, ts):
        self.ts = ts
        self.cutoff_freq = cutoff_freq
        self.pre_out = 0.
        self.tau = self.calc_filter_coef()

    def calc_filter_coef(self):
        w_cut = 2 * np.pi * self.cutoff_freq
        return 1 / w_cut

    def filter(self, data):
        out = (self.tau * self.pre_out + self.ts * data) / (self.tau + self.ts)
        self.pre_out = out
        return out


class Sensor(object):
    def __init__(self):
        self.value = 10
        self.i = 0

    def noise(self):
        return random.randint(-10, 10) * 0.01

    def sense(self):
        self.i += 0.1
        return self.value + self.noise()


if __name__ == "__main__":

    xs = []
    sensors = []
    filters = []

    lpf = LowPassFilter(cutoff_freq=5, ts=0.01)
    sensor = Sensor()

    for i in range(1000):
        z = sensor.sense()
        f = lpf.filter(z)

        xs.append(i)
        sensors.append(z)
        filters.append(f)

    plt.plot(xs, filters)
    plt.scatter(xs, sensors, c="r", s=1)

    plt.show()
