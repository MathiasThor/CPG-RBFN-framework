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

    def step_multi(self, fitness_arr_stab, fitness_arr_coll, fitness_arr_powr, fitness_arr_dist, parameter_arr, noise_arr):
        # Calculate new fitness
        # stability, collisions, power, distance
        W_stab = 0.05
        W_coll = 0.02
        W_powr = 0.1
        W_dist = 0.83

        if W_stab+W_coll+W_powr+W_dist != 1:
            print("weights does not add to one")
            sys.exit()

        parameter_arr_old = parameter_arr

        ###
        # Combine sub-fitness functions
        fitness_arr_comb = [fitness_arr_stab, fitness_arr_coll, fitness_arr_powr, fitness_arr_dist]
        P                = [[0]*self.rollouts, [0]*self.rollouts, [0]*self.rollouts, [0]*self.rollouts]

        for f in range(4):
            max_fitness     = max(fitness_arr_comb[f])
            min_fitness     = min(fitness_arr_comb[f])

            # Run RL algorithm - PI^BB
            S_norm  = [0]*self.rollouts

            # Compute trajectory cost/fitness
            for k in range(self.rollouts):
                S_norm[k] = math.exp(self.h * self.safe_division((fitness_arr_comb[f][k] - min_fitness), (max_fitness - min_fitness)))

            # Compute probability for each roll-out
            for k in range(self.rollouts):
                P[f][k] = S_norm[k] / sum(S_norm)

        # Cost-weighted averaging
        # noise_arr[k]    = [x * P[k] for x in noise_arr[k]]
        for k in range(len(noise_arr)):
            noise_arr[k] = noise_arr[k] * (W_stab*P[0][k] + W_coll*P[1][k] + W_powr*P[2][k] + W_dist*P[3][k])

        # Update policy parameters
        parameter_arr   = list(map(add, parameter_arr, noise_arr[k]))

        # Decay L (λ) / variance as learning progress # Gordon Freeman
        self.h = self.decay * self.h

        return parameter_arr
