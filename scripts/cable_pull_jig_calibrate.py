#!/usr/bin/env python3

import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt

from process_mark10_data import PlotMark10


if __name__ == '__main__':
    mark10 = PlotMark10()

    num_magnets = 3 + 1
    num_exp = 5

    final_force = []
    for magnet in range(num_magnets):
        if magnet == 0:
            continue
        force = []
        for experiment in range(num_exp):
            data = np.array(mark10.load_data(f'magnet_jig_cable_pull/{magnet}_{experiment}.csv'))
            force.append(np.min(data[:, 1]))
        final_force.append(np.mean(force).round(3))
    print(final_force)

    