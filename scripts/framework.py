#!/usr/bin/env python

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
        points = rospy.ServiceProxy('/lidar_processed_readings', ProcessedPoints)
        for pose in self.possible_poses:
            self.pose_belief[pose] = compute_combined_rmse(...)


if __name__ == '__main__':
    rospy.init_node('main_node')
    gs = Alg('../config/grasp_score/mug.yaml')
