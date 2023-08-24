import yaml
import numpy as np
from tabulate import tabulate
import matplotlib.pyplot as plt

from grasp_score import GraspScore
from synthetic_lidar import sample_lidar
from ssd import compute_combined_rmse


class AggregatedScore:
    def __init__(self, config_file):
        with open(config_file, 'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
        self.possible_grasps = list(yaml_data.keys())
        self.possible_poses = list(list(yaml_data.values())[0].keys())
        self.grasp_score = GraspScore(config_file)
        self.belief = None

    def belief_update(self, obs=None, center=None, radius=None, handle_len=None):
        if obs is None:
            self.belief = {}
            for pose in self.possible_poses:
                self.belief[pose] = 1 / len(self.possible_poses)
            print(self.belief)
            return
        rmse = []
        for key in self.possible_poses:
            angle = float(key.split('_')[1])
            rmse.append(
                1 / compute_combined_rmse(obs, center, radius, angle, handle_len) * self.belief[key])
            self.belief[key] = rmse[-1]

        scale = np.sum(np.array(rmse))
        for key in self.possible_poses:
            self.belief[key] /= scale
        print(self.belief)

    def __call__(self, grasp):
        if self.belief is None:
            return None
        res = 0
        for pose in self.possible_poses:
            res += self.belief[pose] * self.grasp_score(grasp, pose)
        return res

    def belief_weighted(self):
        if self.belief is None:
            return None
        # data = np.zeros((len(self.possible_poses), len(self.possible_grasps)))
        # for i, pose in enumerate(self.possible_poses):
        #     b = self.belief[pose]
        #     for j, grasp in enumerate(self.possible_grasps):
        #         data[i, j] = b * self.grasp_score(grasp, pose)
        data = {}
        for pose in self.possible_poses:
            data[pose] = {}
            b = self.belief[pose]
            for grasp in self.possible_grasps:
                data[pose][grasp] = b * self.grasp_score(grasp, pose)
        return data


if __name__ == '__main__':
    agg_score = AggregatedScore('../config/grasp_score/mug.yaml')
    config_file = '../config/object/mug.yaml'
    with open(config_file, 'r') as yaml_file:
        mug_data = yaml.safe_load(yaml_file)
    radius = mug_data['radius']
    center = np.array([mug_data['mug_center']['x'], mug_data['mug_center']['y']])
    handle_len = mug_data['handle_len']
    handle_angle = mug_data['handle_angle']
    lidar_point = np.array([0.0, 0])

    agg_score.belief_update()
    data1 = agg_score.belief_weighted()
    # print(agg_score.belief_weighted())

    samples, _ = sample_lidar(center, radius, handle_len, handle_angle)

    agg_score.belief_update(samples, center, radius, handle_len)
    data2 = agg_score.belief_weighted()
    # print(agg_score.belief_weighted())

    # Extract row keys, column keys, and values for data1
    row_keys1 = list(data1.keys())
    column_keys1 = list(data1[row_keys1[0]].keys())
    values1 = np.array([[data1[row][column] for column in column_keys1] for row in row_keys1])

    # Extract row keys, column keys, and values for data2
    row_keys2 = list(data2.keys())
    column_keys2 = list(data2[row_keys2[0]].keys())
    values2 = np.array([[data2[row][column] for column in column_keys2] for row in row_keys2])

    # Create a figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(6, 4))

    # Create heatmaps for data1 and data2
    cax1 = ax1.matshow(values1, cmap="coolwarm")
    cax2 = ax2.matshow(values2, cmap="coolwarm")

    # Set x and y ticks for both subplots
    for ax in (ax1, ax2):
        ax.set_xticks(np.arange(len(column_keys1)))
        ax.set_yticks(np.arange(len(row_keys1)))
        ax.set_xticklabels(column_keys1)
        ax.set_yticklabels(row_keys1)

    # Add colorbars for both subplots
    cbar1 = fig.colorbar(cax1, ax=ax1)
    cbar2 = fig.colorbar(cax2, ax=ax2)

    # Annotate heatmap cells with values for both subplots
    for ax, values in zip((ax1, ax2), (values1, values2)):
        for i in range(len(row_keys1)):
            for j in range(len(column_keys1)):
                ax.text(j, i, f'{values[i, j]:.2f}', ha='center', va='center', color='w')

    ax1.set_xlabel("Grasps")
    ax1.set_ylabel("Stable Poses")
    ax1.set_title("Heatmap of Grasp Score Before Intervention")

    ax2.set_xlabel("Grasps")
    ax2.set_ylabel("Stable Poses")
    ax2.set_title("Heatmap of Grasp Score After Intervention")

    # Adjust layout spacing between subplots
    plt.tight_layout()

    plt.show()

    # plt.scatter(samples[0, :], samples[1, :], c='b', s=1, label='synthetic samples')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    # plt.legend()
    # plt.show()
