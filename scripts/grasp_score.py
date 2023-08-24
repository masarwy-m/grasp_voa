import yaml
import numpy as np
import matplotlib.pyplot as plt


class GraspScore:
    def __init__(self, config_file):
        with open(config_file, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)
        self.grasp_score = data
        # Extract row keys, column keys, and values
        row_keys = list(data.keys())
        column_keys = list(data[row_keys[0]].keys())
        values = np.array([[data[row][column] for column in column_keys] for row in row_keys]).T

        # Create a heatmap using matplotlib
        fig, ax = plt.subplots()
        cax = ax.matshow(values, cmap="coolwarm")

        # Set x and y ticks
        ax.set_xticks(np.arange(len(row_keys)))
        ax.set_yticks(np.arange(len(column_keys)))
        ax.set_xticklabels(row_keys)
        ax.set_yticklabels(column_keys)

        # Add colorbar
        fig.colorbar(cax)

        # Annotate heatmap cells with values
        for i in range(len(column_keys)):
            for j in range(len(row_keys)):
                ax.text(j, i, f'{values[i, j]:.2f}', ha='center', va='center', color='w')

        plt.xlabel("Grasps")
        plt.ylabel("Stable Poses")
        plt.show()

    def __call__(self, graspID, objectPose):
        return self.grasp_score[graspID][objectPose]


if __name__ == '__main__':
    gs = GraspScore('../config/grasp_score/mug.yaml')
