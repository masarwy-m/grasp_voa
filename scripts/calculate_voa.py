import yaml
import numpy as np
from itertools import product

from mesh_lidar_generator import extract_lidar_readings


class VOA:
    def __init__(self, grasp_score_file, poses_file, mesh_file):
        with open(grasp_score_file, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)
        self.grasp_score = data
        with open(poses_file, 'r') as yaml_file:
            poses = yaml.safe_load(yaml_file)
        # Extract row keys, column keys, and values
        self.grasps = list(data.keys())
        self.obj_poses = list(data[self.grasps[0]].keys())
        # values = np.array([[data[row][column] for column in column_keys] for row in row_keys]).T
        self.sensor_configs = ['Q1', 'Q2', 'Q3', 'Q4']
        self.belief = {}
        for pose in self.obj_poses:
            self.belief[pose] = 1 / len(self.obj_poses)

        self.init_q_star = 0
        self.init_grasp_star = ''
        for grasp in self.grasps:
            q = 0
            for p_k in self.obj_poses:
                q += self.belief[p_k] * self.grasp_score[grasp][p_k]
            if self.init_q_star < q:
                self.init_q_star = q
                self.init_grasp_star = grasp
        print('The best grasp to start with is: ', self.init_grasp_star)

        self.generated_readings = {}
        for sensor_id, pose_id in product(self.sensor_configs, self.obj_poses):
            if not sensor_id in self.generated_readings.keys():
                self.generated_readings[sensor_id] = {}
            pose = poses[pose_id]
            _, _, self.generated_readings[sensor_id][pose_id] = extract_lidar_readings(obj_file_path=mesh_file,
                                                                                       pose=pose,
                                                                                       lidar_height=0.05,
                                                                                       lidar_dist=0.2,
                                                                                       scale=2.5, q=int(sensor_id[1]))

    def similarity(self, ob1, ob2):
        v1 = np.ones((360,))
        v2 = np.ones((360,))
        for degree in ob1.keys():
            v1 = ob1[degree]
        for degree in ob2.keys():
            v2 = ob2[degree]
        norm = np.linalg.norm(v1 - v2)
        return float(np.exp(-norm))

    def belief_update(self, sensor_id, p_i):
        belief = {}
        sensor_id_generate_readings = self.generated_readings[sensor_id]
        normalization = 0
        for p_j in self.obj_poses:
            belief[p_j] = self.similarity(sensor_id_generate_readings[p_i], sensor_id_generate_readings[p_j]) * \
                          self.belief[p_j]
            normalization += belief[p_j]
        for p in belief.keys():
            belief[p] /= normalization
        return belief

    def __call__(self, sensor_id):
        voa = -self.init_q_star
        for p_i in self.obj_poses:
            new_belief = self.belief_update(sensor_id, p_i)
            q_star = 0
            for grasp in self.grasps:
                q = 0
                for p_k in self.obj_poses:
                    q += new_belief[p_k] * self.grasp_score[grasp][p_k]
                q_star = max(q, q_star)
            voa += new_belief[p_i] * q_star
        return voa


if __name__ == '__main__':
    voa_calc = VOA(grasp_score_file='../config/grasp_score/sprayflask.yaml',
                   poses_file='../config/poses/sprayflask_poses.yaml',
                   mesh_file='../data/objects/sprayflask/Sprayflask_800_tex.obj')
    for sensor_id in ['Q1', 'Q2', 'Q3', 'Q4']:
        print(voa_calc(sensor_id))
