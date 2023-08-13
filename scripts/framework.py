#!/usr/bin/env python
import numpy as np
import yaml
import rospy
from grasp_voa.msg import Point2D
from grasp_voa.srv import ProcessedPoints
from ssd import compute_combined_rmse


class Alg:
    def __init__(self, config_file):
        with open(config_file, 'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
        self.possible_grasps = list(yaml_data.keys())
        self.possible_poses = list(list(yaml_data.values())[0].keys())
        self.pose_belief = {}

        rospy.wait_for_service('/lidar_processed_readings')
        points = rospy.ServiceProxy('/lidar_processed_readings', ProcessedPoints)().points
        self.lidar_real_readings = np.zeros((2, len(points)))
        for i in range(len(points)):
            self.lidar_real_readings[0, i] = points[i].x
            self.lidar_real_readings[1, i] = points[i].y

        for pose in self.possible_poses:
            compute_combined_rmse()


if __name__ == '__main__':
    rospy.init_node('main_node')
    gs = Alg('../config/grasp_score/mug.yaml')
