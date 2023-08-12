import numpy as np
from lidar_generator import sample_lidar_readings
from ssd import compute_combined_rmse
import matplotlib.pyplot as plt


class StablePosesBelief:
    def __init__(self, mug_center, mug_radius, handle_len):
        self.mug_center = mug_center
        self.mug_radius = mug_radius
        self.handle_len = handle_len
        num_possible_angles = 8
        possible_angles = np.linspace(0, 360 - (360 / num_possible_angles), num_possible_angles)
        self.possible_poses_belief = {}

        # uniform initializing
        for angle in possible_angles:
            weight = 1 / (4 * num_possible_angles)
            self.possible_poses_belief['up_' + str(angle)] = 3 * weight
            self.possible_poses_belief['down_' + str(angle)] = weight

        print('Initial belief:', self.possible_poses_belief)

    def belief_update(self, observed_points):
        rmse = []
        for key in self.possible_poses_belief:
            angle = float(key.split('_')[1])
            rmse.append(
                1 / compute_combined_rmse(observed_points, self.mug_center, self.mug_radius, angle, self.handle_len) *
                self.possible_poses_belief[key])
            self.possible_poses_belief[key] = rmse[-1]

        scale = np.linalg.norm(np.array(rmse))
        for key in self.possible_poses_belief:
            self.possible_poses_belief[key] /= scale
        print(self.possible_poses_belief)


if __name__ == '__main__':
    circle_center = np.array([2, 2])
    circle_radius = 1.5
    line_length = 1.5
    observing_point = np.array([4, 4])
    belief = StablePosesBelief(circle_center, circle_radius, line_length)
    sampled_points = sample_lidar_readings(circle_center, circle_radius, line_length, 45, observing_point, 40)
    for point in sampled_points:
        plt.plot(point[0], point[1], 'yo', markersize=5)
    plt.show()
    belief.belief_update(sampled_points)
