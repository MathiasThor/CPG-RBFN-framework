#!/usr/bin/env python

import csv
import os
import os.path
import sys
import matplotlib.pyplot as plt
import numpy as np
from drawnow import drawnow

class PLOTTER(object):
    def __init__(self):
        plt.style.use('bmh')
        plt.ion()  # enable interactivity
        fig = plt.figure(num=None, figsize=(12, 6), dpi=80, facecolor='w', edgecolor='k')
        self.i = 0
        self.x = list()
        self.maxstab = list()
        self.avgstab = list()
        self.minstab = list()
        self.maxcoll = list()
        self.avgcoll = list()
        self.mincoll = list()
        self.maxdst = list()
        self.avgdst = list()
        self.mindst = list()
        self.maxpow = list()  # NOW IT IS JUST FITNESS
        self.avgpow = list()  # NOW IT IS JUST FITNESS
        self.minpow = list()  # NOW IT IS JUST FITNESS
        self.plottings = 0
        self.xsize  = 100
        plt.rcParams.update({'font.size': 16})

    def make_fig(self):
        self.plottings = self.plottings+1

        plt.subplot(223)
        plt.plot(np.array(self.x), np.array(self.maxstab), 'r.-')
        plt.plot(np.array(self.x), np.array(self.avgstab), 'g.-')
        plt.plot(np.array(self.x), np.array(self.minstab), 'b.-')
        plt.ylabel('Stability')
        plt.xlabel('Iteration')
        plt.legend(['Max', 'Avg', 'Min'], loc='lower right')

        if self.plottings >= self.xsize:
            plt.xlim((0, self.plottings))
        else:
            plt.xlim((0, self.xsize))
        plt.tight_layout()

        plt.subplot(224)
        plt.plot(np.array(self.x), np.array(self.maxcoll), 'r.-')
        plt.plot(np.array(self.x), np.array(self.avgcoll), 'g.-')
        plt.plot(np.array(self.x), np.array(self.mincoll), 'b.-')
        plt.ylabel('Collision')
        plt.xlabel('Iteration')
        plt.legend(['Max', 'Avg', 'Min'], loc='lower right')

        if self.plottings >= self.xsize:
            plt.xlim((0, self.plottings))
        else:
            plt.xlim((0, self.xsize))
        plt.tight_layout()

        plt.subplot(221)
        plt.plot(np.array(self.x), np.array(self.maxdst), 'r.-')
        plt.plot(np.array(self.x), np.array(self.avgdst), 'g.-')
        plt.plot(np.array(self.x), np.array(self.mindst), 'b.-')
        plt.ylabel('Distance')
        plt.xlabel('Iteration')
        plt.legend(['Max', 'Avg', 'Min'], loc='lower right')

        if self.plottings >= self.xsize:
            plt.xlim((0, self.plottings))
        else:
            plt.xlim((0, self.xsize))
        plt.tight_layout()

        plt.subplot(222)
        plt.plot(np.array(self.x), np.array(self.maxpow), 'r.-')
        plt.plot(np.array(self.x), np.array(self.avgpow), 'g.-')
        plt.plot(np.array(self.x), np.array(self.minpow), 'b.-')
        plt.ylabel('Fitness')
        plt.xlabel('Iteration')
        plt.legend(['Max', 'Avg', 'Min'], loc='lower right')

        if self.plottings >= self.xsize:
            plt.xlim((0, self.plottings))
        else:
            plt.xlim((0, self.xsize))
        plt.tight_layout()

    def plot(self, _fitness_arr_stab, _fitness_arr_coll, _fitness_arr_powr, _fitness_arr_dist):
        # Calculate fitness min, max, and avg.
        max_fitness_stab     = max(_fitness_arr_stab)
        max_fitness_coll     = max(_fitness_arr_coll)
        max_fitness_powr     = max(_fitness_arr_powr)
        max_fitness_dist     = max(_fitness_arr_dist)

        avg_fitness_stab     = sum(_fitness_arr_stab)/len(_fitness_arr_stab)
        avg_fitness_coll     = sum(_fitness_arr_coll)/len(_fitness_arr_coll)
        avg_fitness_powr     = sum(_fitness_arr_powr)/len(_fitness_arr_powr)
        avg_fitness_dist     = sum(_fitness_arr_dist)/len(_fitness_arr_dist)

        min_fitness_stab    = min(_fitness_arr_stab)
        min_fitness_coll    = min(_fitness_arr_coll)
        min_fitness_powr    = min(_fitness_arr_powr)
        min_fitness_dist    = min(_fitness_arr_dist)

        if min_fitness_coll < 0:
            min_fitness_coll = -0.05

        self.x.append(self.i)
        self.maxstab.append(max_fitness_stab)
        self.avgstab.append(avg_fitness_stab)
        self.minstab.append(min_fitness_stab)
        self.maxcoll.append(max_fitness_coll)
        self.avgcoll.append(avg_fitness_coll)
        self.mincoll.append(min_fitness_coll)
        self.maxpow.append(max_fitness_powr)
        self.avgpow.append(avg_fitness_powr)
        self.minpow.append(min_fitness_powr)
        self.maxdst.append(max_fitness_dist)
        self.avgdst.append(avg_fitness_dist)
        self.mindst.append(min_fitness_dist)
        drawnow(self.make_fig)
        self.i = self.i+1

    def plot_existing(self, file_name):

        with open(file_name,'r') as csvfile:
            plots = csv.reader(csvfile, delimiter='\t')
            next(plots, None)

            for i, row in enumerate(plots):
                self.avgstab.append(float(row[4]))
                self.maxstab.append(float(row[5]))
                self.minstab.append(float(row[6]))
                self.avgcoll.append(float(row[7]))
                self.maxcoll.append(float(row[8]))
                self.mincoll.append(float(row[9]))
                self.avgpow.append(float(row[10]))
                self.maxpow.append(float(row[11]))
                self.minpow.append(float(row[12]))
                self.avgdst.append(float(row[13]))
                self.maxdst.append(float(row[14]))
                self.mindst.append(float(row[15]))

        for x in range(len(self.maxstab)):
            self.x.append(self.i)
            self.i = self.i+1

        self.plottings = len(self.maxstab)

        drawnow(self.make_fig)
