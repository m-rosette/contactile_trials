#!/usr/bin/env python3

import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt


class PlotPapillarrayForce:
    def __init__(self) -> None:
        self.num_arrays = 2
        self.num_pillars = 9
        self.num_dim = 3
        self.num_dim_norm = 1    
        self.num_trials = 5    

    def load_data(self, csv_file):
        return pd.read_csv(csv_file)

    def extract_columns(self, data, header_starts_with=('0_fX', '0_fY', '0_fZ')):
        # Convert all column names to strings and then filter columns
        filtered_columns = [col for col in map(str, data.columns) if col.startswith(header_starts_with)]

        # Extract the filtered columns
        extracted_data = data[filtered_columns]

        return extracted_data

    def norm_time(self, timestamp):
        init_time = timestamp[0]
        return timestamp - init_time

    def get_force_data(self, data):
        pillar_force = np.zeros((self.num_arrays, self.num_pillars, data.shape[0], self.num_dim))
        for array in range(self.num_arrays):
            for pillar in range(self.num_pillars):
                column_search = (f'{array}_fX_{pillar}', f'{array}_fY_{pillar}', f'{array}_fZ_{pillar}')
                pillar_force[array, pillar, :, :] = self.extract_columns(data, column_search)
        return pillar_force
    
    def rearrange_force(self, data):
        # 6=0, 7=1, 8=2
        force_data_copy = data.copy()
        force_data_copy[1, [6, 0], :, :] = force_data_copy[1, [0, 6], :, :]
        force_data_copy[1, [7, 1], :, :] = force_data_copy[1, [1, 7], :, :]
        force_data_copy[1, [8, 2], :, :] = force_data_copy[1, [2, 8], :, :]
        return force_data_copy
    
    def force_stats(self, trial_num, directory):
        average_force = None  # Placeholder for storing the cumulative force values
        all_trial_forces = []

        # Loop through each file in the directory
        for _ in range(self.num_trials):
            filename = os.path.join(directory, f'{trial_num}.csv')

            # Load data from the file
            data = self.load_data(filename)

            # Extract force data and normalize if needed
            pillar_force = self.get_force_data(data)
            pillar_force = self.rearrange_force(pillar_force)

            all_trial_forces.append(pillar_force)

            # Calculate average force values
            if average_force is None:
                average_force = pillar_force
            else:
                average_force += pillar_force

            trial_num += 1

        # Calculate the average across trials
        average_force /= self.num_trials

        # Calculate standard deviation
        all_trial_forces = np.array(all_trial_forces)
        std_dev_force = np.std(all_trial_forces, axis=0)

        return average_force, std_dev_force

    def get_norm_force(self, data):
        norm_pillar_force = np.zeros((self.num_arrays, self.num_pillars, data.shape[2], self.num_dim_norm))
        for array in range(self.num_arrays):
            for pillar in range(self.num_pillars):
                data_to_norm = data[array, pillar, :, :]
                norm_data = np.linalg.norm(data_to_norm, axis=1)
                # Reshape the norm result to match the target shape (51, 1)
                norm_data = norm_data.reshape(norm_data.shape[0], 1)
                norm_pillar_force[array, pillar, :, :] = norm_data
        return norm_pillar_force

    def norm_trial_stats(self, trial_num, directory):
        average_force = None  # Placeholder for storing the cumulative force values
        all_trial_forces = []

        # Loop through each file in the directory
        for _ in range(self.num_trials):
            filename = os.path.join(directory, f'{trial_num}.csv')

            # Load data from the file
            data = self.load_data(filename)

            # Extract force data and normalize if needed
            pillar_force = self.get_force_data(data)
            norm_pillar_force = self.get_norm_force(pillar_force)

            all_trial_forces.append(norm_pillar_force)

            # Calculate average force values
            if average_force is None:
                average_force = norm_pillar_force
            else:
                average_force += norm_pillar_force

            trial_num += 1

        # Calculate the average across trials
        average_force /= self.num_trials

        # Calculate standard deviation
        all_trial_forces = np.array(all_trial_forces)
        std_dev_force = np.std(all_trial_forces, axis=0)

        return average_force, std_dev_force
    
    def plot_norm_force(self, timestamp, norm_pillar_force, std_data=None, ax=None):
        if ax is None:
            fig, axs = plt.subplots(1, self.num_arrays, figsize=(8 * self.num_arrays, 6), sharex=True)
        else:
            axs = ax  # Use provided axes

        timestamp = np.squeeze(timestamp)

        for array in range(self.num_arrays):
            if ax is None:
                axs[array].set_ylabel('Normalized Force')
                axs[array].set_xlabel('Time (s)')
                axs[array].set_title(f'Array {array} - Normalized Force Data Over Time')

            for pillar in range(self.num_pillars):
                # Get the normalized force data for each array and pillar
                norm_force = norm_pillar_force[array, pillar, :, 0]  # Assuming the dimension is (51, 1)

                # Plot the normalized force over time with normalized timestamps
                if ax is None:
                    axs[array].plot(timestamp, norm_force, label=f'Pillar {pillar}')

                    if std_data is not None:
                        std_dev = std_data[array, pillar, :, 0]
                        axs[array].fill_between(timestamp, norm_force - std_dev, norm_force + std_dev, alpha=0.3)

                    axs[array].legend(loc='upper left')
                else:
                    ax[array].plot(timestamp, norm_force, label=f'Pillar {pillar}')

                    if std_data is not None:
                        std_dev = std_data[array, pillar, :, 0]
                        ax[array].fill_between(timestamp, norm_force - std_dev, norm_force + std_dev, alpha=0.3)

        if ax is None:
            plt.tight_layout()
            plt.show()

    # def plot_xyz_force(self, timestamp, avg_force, stdev_force):
    #     # Assuming 'data' contains x, y, z values for each subplot
    #     num_rows = 3
    #     num_cols = 3
    #     fig, axs = plt.subplots(num_rows, num_cols, figsize=(14, 12))

    #     # Flatten the axis array for easy iteration
    #     axs = axs.flatten()
    #     timestamp = np.squeeze(timestamp)

    #     # Define the order of subplots
    #     subplot_order = [6, 3, 0, 7, 4, 1, 8, 5, 2]

    #     for array in range(self.num_arrays):
    #         if array == 0 :
    #             x_color = [135/255, 206/255, 235/255] # Light blue
    #             y_color = [60/255, 179/255, 113/255] # Light green
    #             z_color = [255/255, 140/255, 10/255] # Light orange
    #         if array == 1:
    #             x_color = [30/255, 144/255, 255/255] # Dark blue
    #             y_color = [50/255, 205/255, 50/255] # Dark green
    #             z_color = [255/255, 127/255, 120/255] # Dark orange
    #         for pillar, subplot_idx in enumerate(subplot_order):
    #             x = avg_force[array, pillar, :, 0]
    #             y = avg_force[array, pillar, :, 1]
    #             z = avg_force[array, pillar, :, 2]
    #             axs[pillar].plot(timestamp, x, label=f'X_{array}', color=x_color)
    #             axs[pillar].plot(timestamp, y, label=f'Y_{array}', color=y_color)
    #             axs[pillar].plot(timestamp, z, label=f'Z_{array}', color=z_color)

    #             axs[pillar].fill_between(timestamp, x - stdev_force[array, pillar, :, 0], x + stdev_force[array, pillar, :, 0], alpha=0.3, color=x_color)
    #             axs[pillar].fill_between(timestamp, y - stdev_force[array, pillar, :, 1], y + stdev_force[array, pillar, :, 1], alpha=0.3, color=y_color)
    #             axs[pillar].fill_between(timestamp, z - stdev_force[array, pillar, :, 2], z + stdev_force[array, pillar, :, 2], alpha=0.3, color=z_color)

    #             axs[pillar].set_title(f'Pillar {pillar}')
    #             axs[pillar].legend()
    #             axs[pillar].set_xlabel('Time')
    #             axs[pillar].set_ylabel('Force')

    #     plt.tight_layout()
    #     plt.show()
    def plot_xyz_force(self, timestamp, avg_force, stdev_force):
        # Assuming 'data' contains x, y, z values for each subplot
        num_rows = 3
        num_cols = 3
        fig, axs = plt.subplots(num_rows, num_cols, figsize=(14, 12))

        # Flatten the axis array for easy iteration
        axs = axs.flatten()
        timestamp = np.squeeze(timestamp)

        # Define the order of subplots
        subplot_order = [6, 3, 0, 7, 4, 1, 8, 5, 2]

        for array in range(self.num_arrays):
            if array == 0 :
                x_color = [135/255, 206/255, 235/255] # Light blue
                y_color = [60/255, 179/255, 113/255] # Light green
                z_color = [255/255, 140/255, 10/255] # Light orange
            if array == 1:
                x_color = [30/255, 144/255, 255/255] # Dark blue
                y_color = [50/255, 205/255, 50/255] # Dark green
                z_color = [255/255, 127/255, 120/255] # Dark orange
            for pillar, subplot_idx in enumerate(subplot_order):
                x = avg_force[array, subplot_idx, :, 0]
                y = avg_force[array, subplot_idx, :, 1]
                z = avg_force[array, subplot_idx, :, 2]
                axs[pillar].plot(timestamp, x, label=f'X_{array}', color=x_color)
                axs[pillar].plot(timestamp, y, label=f'Y_{array}', color=y_color)
                axs[pillar].plot(timestamp, z, label=f'Z_{array}', color=z_color)

                axs[pillar].fill_between(timestamp, x - stdev_force[array, subplot_idx, :, 0], x + stdev_force[array, subplot_idx, :, 0], alpha=0.3, color=x_color)
                axs[pillar].fill_between(timestamp, y - stdev_force[array, subplot_idx, :, 1], y + stdev_force[array, subplot_idx, :, 1], alpha=0.3, color=y_color)
                axs[pillar].fill_between(timestamp, z - stdev_force[array, subplot_idx, :, 2], z + stdev_force[array, subplot_idx, :, 2], alpha=0.3, color=z_color)

                axs[pillar].set_title(f'Pillar {subplot_idx}')
                axs[pillar].legend()
                axs[pillar].set_xlabel('Time')
                axs[pillar].set_ylabel('Force')

                axs[pillar].set_ylim([-3.5, 3.5])

        plt.tight_layout()
        plt.show()



if __name__ == '__main__':
    pltforce = PlotPapillarrayForce()
    
    filename = 'data_01_05_24/4.csv'
    data = pltforce.load_data(filename)
    timestamp = np.array(pltforce.extract_columns(data, ('timestamp')))
    timestamp = pltforce.norm_time(timestamp)

    # pillar_force = pltforce.get_force_data(data)
    # pillar_force = pltforce.rearrange_force(pillar_force)

    data_directory = 'data_01_05_24'  # Change this to your data directory path
    trial_num = 0
    avg_pillar_force, std_pillar_force = pltforce.force_stats(trial_num, data_directory)
    pltforce.plot_xyz_force(timestamp, avg_pillar_force, std_pillar_force)

    # pltforce.plot_xyz_force(pillar_force, timestamp)  # Assuming you want to plot the data for the first array

    # # pillar_force = get_force_data(data)
    # # norm_pillar_force, std_pillar_force = get_norm_force(pillar_force)

    # data_directory = 'data'  # Change this to your data directory path
    # trial_num = 20
    # norm_pillar_force, std_pillar_force = pltforce.norm_trial_stats(trial_num, data_directory)

    # pltforce.plot_norm_force(timestamp, norm_pillar_force, std_pillar_force)

