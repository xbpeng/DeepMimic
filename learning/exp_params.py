import json
import numpy as np
import util.math_util as MathUtil

class ExpParams(object):
    RATE_KEY = 'Rate'
    NOISE_KEY = 'Noise'

    def __init__(self):
        self.rate = 0.2
        self.noise = 0.1
        return

    def __str__(self):
        str = ''
        str += '{}: {:.2f}\n'.format(self.RATE_KEY, self.rate)
        str += '{}: {:.2f}\n'.format(self.NOISE_KEY, self.noise)
        return str

    def load(self, json_data):
        if (self.RATE_KEY in json_data):
            self.rate = json_data[self.RATE_KEY]

        if (self.NOISE_KEY in json_data):
            self.noise = json_data[self.NOISE_KEY]

        return

    def lerp(self, other, t):
        lerp_params = ExpParams()
        lerp_params.rate = MathUtil.lerp(self.rate, other.rate, t)
        lerp_params.noise = MathUtil.lerp(self.noise, other.noise, t)
        return lerp_params