import numpy as np
import copy
from util.logger import Logger
import util.math_util as MathUtil

class ReplayBufferRandStorage(object):
    def __init__(self, buffer_size):
        assert buffer_size > 0

        self._buffer_size = buffer_size
        self._curr_size = 0
        self._total_count = 0
        self._buffer = None

        self.clear()
        return

    def sample(self, n):
        curr_size = self.get_current_size()
        assert curr_size > 0
        idx = np.random.randint(0, curr_size, size=n)
        return idx

    def get(self, idx):
        return self._buffer[idx]

    def store(self, data):
        n = len(data)
        if (n > 0):
            if self._buffer is None:
                self._init_buffer(data)

            idx = self._request_idx(n)
            self._buffer[idx] = data
                
            self._curr_size = min(self._curr_size + n, self._buffer_size)
            self._total_count += n
        return

    def is_full(self):
        return self._curr_size >= self._buffer_size

    def clear(self):
        self._curr_size = 0
        self._total_count = 0
        return
    
    def get_buffer_size(self):
        return self._buffer_size
    
    def get_current_size(self):
        return self._curr_size

    def _init_buffer(self, data):
        dtype = data[0].dtype
        shape = [self._buffer_size] + list(data[0].shape)
        self._buffer = np.zeros(shape, dtype=dtype)
        return

    def _request_idx(self, n):
        assert n < self._buffer_size # bad things can happen if path is too long
        curr_size = self.get_current_size()

        idx = []
        if (not self.is_full()):
            start_idx = curr_size
            end_idx = min(self._buffer_size, start_idx + n)
            idx = list(range(start_idx, end_idx))

        remainder = n - len(idx)
        if (remainder > 0):
            rand_idx = list(np.random.choice(curr_size, remainder, replace=False))
            idx += rand_idx

        return idx