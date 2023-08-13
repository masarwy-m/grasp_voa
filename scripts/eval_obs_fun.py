#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import yaml
import matplotlib.pyplot as plt
import csv

from synthetic_lidar import sample_lidar


class LidarReadings:
    def __init__(self, config_file):
        rospy.init_node('lidar_eval_node', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.msg = None
        with open(config_file, 'r') as yaml_file:
            mug_data = yaml.safe_load(yaml_file)
        self.radius = mug_data['radius']
        self.center = np.array([mug_data['mug_center']['x'], mug_data['mug_center']['y']])
        self.handle_len = mug_data['handle_len']
        self.handle_angle = mug_data['handle_angle']

    def lidar_callback(self, msg):
        if self.msg is None:
            self.msg = msg
            samples, readings = sample_lidar(self.center, self.radius, self.handle_len, self.handle_angle)
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
                    if 0.25 <= x < 0.4 and -0.05 < y < 0.05:
                        X.append(x)
                        Y.append(y)
                        if i in readings.keys():
                            res.append((i, dis, readings[i]))
                angle += inc
            with open(str(self.handle_angle) + '.csv', 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerows(res)
            plt.scatter(samples[0, :], samples[1, :], c='b', s=1, label='synthetic samples')
            plt.scatter(X, Y, c='r', s=1, label='real data')  # 's' controls the size of the points
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('Real Data vs. Synthetic Data, Handle Angle is ' + str(self.handle_angle))
            plt.legend()
            plt.show()


if __name__ == '__main__':
    lr = LidarReadings('../config/object/mug.yaml')
    rospy.spin()
