import numpy as np
import math
import yaml
import matplotlib.pyplot as plt


def closest_point_to_origin(point1, point2):
    distance1 = math.sqrt(point1[0] ** 2 + point1[1] ** 2)
    distance2 = math.sqrt(point2[0] ** 2 + point2[1] ** 2)

    if distance1 < distance2:
        return point1
    else:
        return point2


def find_segment_intersection(slope, segment_point1, segment_point2):
    # Calculate the intersection point
    slope_2 = (segment_point2[1] - segment_point1[1]) / (segment_point2[0] - segment_point1[0])
    x_intersection = (segment_point1[1] - slope_2 * segment_point1[0]) / (slope - slope_2)
    y_intersection = slope * x_intersection

    # Check if intersection point lies within the segment's bounding box
    if (
            min(segment_point1[0], segment_point2[0]) <= x_intersection <= max(segment_point1[0], segment_point2[0]) and
            min(segment_point1[1], segment_point2[1]) <= y_intersection <= max(segment_point1[1], segment_point2[1])
    ):
        return x_intersection, y_intersection
    else:
        return None


def find_circle_intersection(circle_center, circle_radius, line_slope):
    # Unpack the circle center coordinates
    h, k = circle_center

    # Slope of the line passing through the origin
    m = line_slope

    # Quadratic coefficients for the intersection equation
    a = 1 + m ** 2
    b = -2 * (m * k + h)
    c = h ** 2 + k ** 2 - circle_radius ** 2

    # Calculate the discriminant
    discriminant = b ** 2 - 4 * a * c

    if discriminant >= 0:
        # Calculate both intersection points
        x1 = (-b + math.sqrt(discriminant)) / (2 * a)
        y1 = m * x1

        x2 = (-b - math.sqrt(discriminant)) / (2 * a)
        y2 = m * x2

        # Calculate distances to the origin
        distance1 = math.sqrt(x1 ** 2 + y1 ** 2)
        distance2 = math.sqrt(x2 ** 2 + y2 ** 2)

        # Return the intersection point closest to the origin
        if distance1 < distance2:
            return x1, y1
        else:
            return x2, y2
    else:
        return None  # No intersection


def sample_lidar(mug_center, mug_radius, handle_len, handle_angle):
    angle_radians = np.radians(handle_angle)
    endpoint_on_circle = mug_center + mug_radius * np.array([np.cos(angle_radians), np.sin(angle_radians)])
    endpoint_outside_circle = endpoint_on_circle + handle_len * np.array(
        [np.cos(angle_radians), np.sin(angle_radians)])

    samples = []
    with open('../config/lidar.yaml', 'r') as yaml_file:
        lidar_config = yaml.safe_load(yaml_file)
    inc = lidar_config['angle_increment']
    angle = lidar_config['angle_min']
    angle_dis = {}
    degree = 0
    while angle <= lidar_config['angle_max']:
        p1 = find_circle_intersection(mug_center, mug_radius, math.tan(angle))
        p2 = find_segment_intersection(math.tan(angle), endpoint_on_circle, endpoint_outside_circle)
        if p1 is not None and p2 is not None:
            samples.append(closest_point_to_origin(p1, p2))
            angle_dis[degree] = np.sqrt(samples[-1][0] ** 2 + samples[-1][1] ** 2)
        elif p1 is not None:
            samples.append(p1)
            angle_dis[degree] = np.sqrt(samples[-1][0] ** 2 + samples[-1][1] ** 2)
        elif p2 is not None:
            samples.append(p2)
            angle_dis[degree] = np.sqrt(samples[-1][0] ** 2 + samples[-1][1] ** 2)
        angle += inc
        degree += 1
    return np.array(samples).T, angle_dis


if __name__ == '__main__':
    config_file = '../config/object/mug.yaml'
    with open(config_file, 'r') as yaml_file:
        mug_data = yaml.safe_load(yaml_file)
    radius = mug_data['radius']
    center = np.array([mug_data['mug_center']['x'], mug_data['mug_center']['y']])
    handle_len = mug_data['handle_len']
    handle_angle = mug_data['handle_angle']
    lidar_point = np.array([0, 0])
    samples, _ = sample_lidar(center, radius, handle_len, handle_angle)
    print(samples.shape)
    plt.scatter(samples[0, :], samples[1, :], s=1)  # 's' controls the size of the points
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Lidar Readings as 2D Points')
    plt.show()
