#!/usr/bin/env python3

import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt


class PlotMark10:
    def __init__(self) -> None:
        self.num_arrays = 2
        self.num_pillars = 9
        self.num_dim = 3
        self.num_dim_norm = 1  
        self.num_trials = 5    
        self.len_data = 240 -1 

    def load_data(self, csv_file):
        data = pd.read_csv(csv_file, sep='\t', skiprows=5)
        return data.values

    def trial_stats(self, trial_num, directory):
        avg_force = None  # Placeholder for storing the cumulative force values
        avg_travel = None  # Placeholder for storing the cumulative force values
        all_trial_forces = []
        all_trial_travel = []

        # Loop through each file in the directory
        for _ in range(self.num_trials):
            filename = os.path.join(directory, f'{trial_num}_mark10.csv') 

            # Load data from the file
            data = self.load_data(filename)

            # Sometimes the mark10 produced an additional datapoint - limit the length of the dataset
            if len(data) > self.len_data:
                data = data[:self.len_data]

            force = data[:, 1]
            travel = data[:, 2]
            timestamp = data[:, 3]

            all_trial_forces.append(force)
            all_trial_travel.append(travel)

            # Calculate avg force values
            if avg_force is None:
                avg_force = force
            else:
                avg_force += force

            # Calculate avg travel values
            if avg_travel is None:
                avg_travel = travel
            else:
                avg_travel += travel

            trial_num += 1

        # Calculate the avg across trials
        avg_force /= self.num_trials
        avg_travel /= self.num_trials

        # Calculate standard deviation
        all_trial_forces = np.array(all_trial_forces)
        all_trial_travel = np.array(all_trial_travel)
        std_dev_force = np.std(all_trial_forces, axis=0)
        std_dev_travel = np.std(all_trial_travel, axis=0)

        return timestamp, avg_force, std_dev_force, avg_travel, std_dev_travel
    
    def plot_avg_force(self, timestamp, avg_force, avg_travel, std_force=None, std_travel=None, ax=None):
        if ax is None:
            fig, axs = plt.subplots(1, 2, figsize=(16, 6), sharex=True)
        else:
            axs = ax  # Use provided axes

        timestamp = np.squeeze(timestamp)

        axs[0].set_ylabel('Force (N)')
        axs[0].set_xlabel('Time (s)')
        axs[0].set_title(f'Average Mark10 Force')

        axs[1].set_ylabel('Travel (in)')
        axs[1].set_xlabel('Time (s)')
        axs[1].set_title(f'Average Mark10 Travel')

        # Plot force
        axs[0].plot(timestamp, avg_force, label=f'Avg Force')

        # Plot travel
        axs[1].plot(timestamp, avg_travel, label=f'Avg Travel')

        if std_force is not None:
            axs[0].fill_between(timestamp, avg_force - std_force, avg_force + std_force, alpha=0.3, label=f'Std Force')
        if std_travel is not None:
            axs[1].fill_between(timestamp, avg_travel - std_travel, avg_travel + std_travel, alpha=0.3, label=f'Std Travel')

        axs[0].legend(loc='upper left')
        axs[1].legend(loc='upper left')

        if ax is None:
            plt.tight_layout()
            plt.show()


if __name__ == '__main__':
    pltmark10 = PlotMark10()
    
    # filename = 'mark10_data/15_mark10.csv'
    # data = pltmark10.load_data(filename)

    directory = 'mark10_data'
    trial_num = 45
    timestamp, avg_force, std_force, avg_travel, std_travel = pltmark10.trial_stats(trial_num, directory)
    pltmark10.plot_avg_force(timestamp, avg_force, avg_travel, std_force, std_travel)



