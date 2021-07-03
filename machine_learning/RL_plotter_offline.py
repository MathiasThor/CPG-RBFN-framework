#!/usr/bin/env python

import csv
import os
import os.path
import sys

import matplotlib.pyplot as plt
import numpy as np


def plot(_maxfit, _avgfit, _minfit, _maxdst, _avgdst, _mindst, _maxpow, _avgpow, _minpow):
    x = list(range(len(_maxfit)))
    maxfit = [float(i) for i in _maxfit]
    avgfit = [float(i) for i in _avgfit]
    minfit = [float(i) for i in _minfit]
    maxdst = [float(i) for i in _maxdst]
    avgdst = [float(i) for i in _avgdst]
    mindst = [float(i) for i in _mindst]
    maxpow = [float(i) for i in _maxpow]
    avgpow = [float(i) for i in _avgpow]
    minpow = [float(i) for i in _minpow]

    plt.style.use('bmh')
    fig = plt.figure(num=None, figsize=(12, 6), dpi=80, facecolor='w', edgecolor='k')

    plt.rcParams.update({'font.size': 16})

    plt.subplot(212)
    plt.plot(np.array(x), np.array(maxfit), 'r.-')
    plt.plot(np.array(x), np.array(avgfit), 'g.-')
    plt.plot(np.array(x), np.array(minfit), 'b.-')
    plt.ylabel('Fitness Value')
    plt.xlabel('Iteration')
    plt.legend(['Max', 'Avg', 'Min'], loc='best')
    plt.ylim((0, 2))
    plt.tight_layout()

    plt.subplot(221)
    plt.plot(np.array(x), np.array(maxdst), 'r.-')
    plt.plot(np.array(x), np.array(avgdst), 'g.-')
    plt.plot(np.array(x), np.array(mindst), 'b.-')
    plt.ylabel('Distance')
    plt.xlabel('Iteration')
    plt.legend(['Max', 'Avg', 'Min'], loc='best')
    plt.ylim((0, 2.5))
    plt.tight_layout()

    plt.subplot(222)
    plt.plot(np.array(x), np.array(maxpow), 'r.-')
    plt.plot(np.array(x), np.array(avgpow), 'g.-')
    plt.plot(np.array(x), np.array(minpow), 'b.-')
    plt.ylabel('Power')
    plt.xlabel('Iteration')
    plt.legend(['Max', 'Avg', 'Min'], loc='best')
    plt.ylim((0, 2.5))
    plt.tight_layout()


    plt.show()


def main(argv):
    if len(sys.argv) > 2:
        if sys.argv[1] == "-p" or sys.argv[1] == "--path":
            if sys.argv[2] != "":
                exists = os.path.isfile(sys.argv[2])
                if exists:
                    with open(sys.argv[2]) as f:
                        reader = csv.reader(f, delimiter="\t")
                        d = list(reader)
                else:
                    print("Files does not exists")
                    sys.exit()
        else:
            print("Please provide a valid arguments...")
            sys.exit()
    else:
        print("Please provide a valid arguments...")
        sys.exit()

    d = np.delete(d, 0, 0)
    plot(np.array(d)[:, 0], np.array(d)[:, 1], np.array(d)[:, 2], np.array(d)[:, 3], np.array(d)[:, 4], np.array(d)[:, 5], np.array(d)[:, 6], np.array(d)[:, 7], np.array(d)[:, 8])

if __name__ == '__main__':
    main(sys.argv[1:])
