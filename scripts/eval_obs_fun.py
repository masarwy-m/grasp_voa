#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import yaml
import matplotlib.pyplot as plt
import csv

from mesh_lidar_generator import extract_lidar_readings
from synthetic_lidar import sample_lidar


class LidarReadings:
    def __init__(self, pose, mesh_file, dist, scale, pose_id, obj):
        rospy.init_node('lidar_eval_node', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.msg = None
        self.dist = dist
        self.pose = pose
        self.pose_id = pose_id
        self.mesh_file = mesh_file
        self.scale = scale
        self.obj = obj

    def lidar_callback(self, msg):
        if self.msg is None:
            self.msg = msg
            # samples, readings = sample_lidar(self.center, self.radius, self.handle_len, self.handle_angle)
            samples, _, readings = extract_lidar_readings(obj_file_path=self.mesh_file, pose=self.pose,
                                                          lidar_height=0.014, lidar_dist=self.dist,
                                                          scale=self.scale)
            angle = msg.angle_min
            inc = msg.angle_increment
            lidar_readings = np.array(msg.ranges)
            X = []
            Y = []
            res = []
            for i, dis in enumerate(lidar_readings):
                if i == 359:
                    break
                x = dis * np.cos(angle)
                y = dis * np.sin(angle)
                if dis != np.inf:
                    if self.dist - 0.20 <= x < self.dist + 0.20 and -0.1 < y < 0.1:
                        X.append(x)
                        Y.append(y)
                        if i in readings.keys():
                            res.append((i, dis, readings[i]))
                angle += inc

            with open('../results/' + self.obj + '/' + str(self.dist) + '_' + str(self.pose_id) + '.csv', 'w',
                      newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerows(res)
            res = []
            for i, dis in enumerate(lidar_readings):
                if i == 359:
                    break
                x = dis * np.cos(np.radians(angle))
                y = dis * np.sin(np.radians(angle))
                res.append((i, dis if dis != np.inf and 0.1 <= x < 0.8 and -0.1 < y < 0.1 else None,
                            readings[i] if i in readings.keys() else None))
            with open(
                    '../results/' + self.obj + '/data_' + str(self.dist) + '_' + str(self.pose_id) + '.csv', 'w',
                    newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerows(res)
            plt.scatter(samples[:, 0], samples[:, 1], c='b', s=1, label='synthetic samples')
            plt.scatter(X, Y, c='r', s=1, label='real data')  # 's' controls the size of the points
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('Real Data vs. Synthetic Data')
            plt.legend()
            plt.show()


if __name__ == '__main__':
    mesh_file = '../data/objects/sprayflask/sprayflask.obj'
    dist = 0.3
    scale = 3
    pose_id = 2
    obj = 'sprayflask'
    with open('../config/poses/sprayflask_poses.yaml', 'r') as yaml_file:
        pose = yaml.safe_load(yaml_file)['pose_' + str(pose_id)]
    lr = LidarReadings(mesh_file=mesh_file, dist=dist, scale=scale, pose_id=pose_id,
                       pose=pose, obj=obj)
    rospy.spin()
