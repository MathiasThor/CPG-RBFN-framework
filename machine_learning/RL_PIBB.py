#!/usr/bin/env python

import sys
import math
from operator import add

class PIBB(object):
    def __init__(self, _rollouts, _h, _decay):
        self.rollouts   = _rollouts
        self.h          = _h
        self.decay      = _decay
        self.p = [0]*self.rollouts

    def get_p(self):
        return self.p

    def get_h(self):
        return self.h

    def safe_division(self, n, d):
        return n / d if d else 0

    def step(self, fitness_arr, parameter_arr, noise_arr):
        # Calculate fitness min, max, and avg.
        max_fitness     = max(fitness_arr)
        min_fitness     = min(fitness_arr)

        # Run RL algorithm - PI^BB
        s_norm  = [0]*self.rollouts
        self.p  = [0]*self.rollouts

        # Compute trajectory cost/fitness
        for k in range(self.rollouts):
            s_norm[k] = math.exp(self.h * self.safe_division((fitness_arr[k] - min_fitness), (max_fitness - min_fitness)))

        # Compute probability for each roll-out
        for k in range(self.rollouts):
            self.p[k]            = s_norm[k] / sum(s_norm)
            # Cost-weighted averaging
            noise_arr[k]    = [x * self.p[k] for x in noise_arr[k]]
            # Update policy parameters
            parameter_arr   = list(map(add, parameter_arr, noise_arr[k]))

        # Decay h (1/λ) / variance as learning progress
        self.h = self.decay * (1/self.h)
        self.h = (1/self.h)

        return parameter_arr
