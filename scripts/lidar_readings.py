#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from grasp_voa.srv import ProcessedPointsResponse, ProcessedPoints
from grasp_voa.msg import Point2D


class LidarReadings:
    def __init__(self):
        rospy.init_node('lidar_reader_node', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Service('/lidar_processed_readings', ProcessedPoints, self.get_readings)
        self.lidar_readings = None
        self.plot = True
        self.readings_2d = np.zeros((2, 360))
        self.clean_readings = []

    def get_readings(self, req):
        res = ProcessedPointsResponse()
        res.points = [Point2D(x=self.clean_readings[i][0], y=self.clean_readings[i][1]) for i in
                      range(len(self.clean_readings))]
        return res

    def lidar_callback(self, msg: LaserScan):
        if self.lidar_readings is None:
            angle = msg.angle_min
            inc = msg.angle_increment
            self.lidar_readings = np.array(msg.ranges)
            for i, dis in enumerate(self.lidar_readings):
                if dis != np.inf:
                    self.readings_2d[0, i] = dis * np.cos(angle)
                    self.readings_2d[1, i] = dis * np.sin(angle)
                    if 0.25 <= self.readings_2d[0, i] < 0.4 and -0.05 < self.readings_2d[1, i] < 0.05:
                        self.clean_readings.append([self.readings_2d[0, i], self.readings_2d[1, i]])
                    angle += inc
            if self.plot:
                plt.scatter(self.readings_2d[0, :], self.readings_2d[1, :], s=1)  # 's' controls the size of the points
                plt.xlabel('X')
                plt.ylabel('Y')
                plt.title('Lidar Readings as 2D Points')

                plt.show()


if __name__ == '__main__':
    lr = LidarReadings()
    rospy.spin()
