import numpy as np


def find_tangent_points(circle_center, circle_radius, point_outside):
    # Calculate the distance between the circle's center and the point outside
    distance_to_center = np.linalg.norm(circle_center - point_outside)

    # Calculate the angle between the radius and the line connecting the circle's center and the point outside
    angle_to_center = np.arctan2(circle_center[1] - point_outside[1], circle_center[0] - point_outside[0])

    # Calculate the length of the line from the point outside to the tangent points
    distance_to_tangent = np.sqrt(distance_to_center ** 2 - circle_radius ** 2)

    # Calculate the angles of the tangent lines
    tangent_angle1 = angle_to_center + np.arcsin(circle_radius / distance_to_center)
    tangent_angle2 = angle_to_center - np.arcsin(circle_radius / distance_to_center)

    # Calculate the tangent points
    tangent_point_1 = point_outside + distance_to_tangent * np.array([np.cos(tangent_angle1), np.sin(tangent_angle1)])
    tangent_point_2 = point_outside + distance_to_tangent * np.array([np.cos(tangent_angle2), np.sin(tangent_angle2)])

    return tangent_point_1, tangent_point_2, tangent_angle1, tangent_angle2


def plot_circle_and_tangent_points(circle_center, circle_radius, angle1, angle2, num_sam):
    arc_angles = np.linspace(angle1, angle2, num_sam)
    return np.array(
        [[circle_center[0] + circle_radius * np.cos(angle), circle_center[1] + circle_radius * np.sin(angle)] for angle
         in arc_angles])


def sample_handle(circle_center, circle_radius, angle_degrees, line_length, num_sam):
    # Convert the angle to radians
    angle_radians = np.radians(angle_degrees)

    # Calculate the endpoint of the line segment on the circle's perimeter
    endpoint_on_circle = circle_center + circle_radius * np.array([np.cos(angle_radians), np.sin(angle_radians)])

    # Calculate the endpoint of the line segment outside the circle
    endpoint_outside_circle = endpoint_on_circle + line_length * np.array(
        [np.cos(angle_radians), np.sin(angle_radians)])

    # Sample points along the line segment using linear interpolation
    return np.linspace(endpoint_on_circle, endpoint_outside_circle, num_sam)


def add_noise(points, noise_level=0.05):
    noisy_points = points + noise_level * np.random.randn(*points.shape)
    return noisy_points


def sample_lidar_readings(mug_center, mug_radius, handle_len, handle_angle, lidar_point, num_samples):
    # TODO add noise to mug_center and handle_angle

    # Tangent points and their angles
    tangent_point1, tangent_point2, _, _ = find_tangent_points(mug_center, mug_radius, lidar_point)

    angle1_rc = np.arctan2(tangent_point1[1] - mug_center[1], tangent_point1[0] - mug_center[0])
    angle2_rc = np.arctan2(tangent_point2[1] - mug_center[1], tangent_point2[0] - mug_center[0])

    samp_on_arc = int(num_samples * 0.625)
    samp_on_handle = num_samples - samp_on_arc

    # sample points on the visible arc
    samples = plot_circle_and_tangent_points(mug_center, mug_radius, angle1_rc, angle2_rc, samp_on_arc)

    if max(np.degrees(angle1_rc), np.degrees(angle2_rc)) > handle_angle > min(np.degrees(angle1_rc),
                                                                              np.degrees(angle2_rc)):
        # Plot the circle with the perpendicular line segment
        samples = np.vstack((samples, sample_handle(mug_center, mug_radius, handle_angle, handle_len, samp_on_handle)))

    return add_noise(samples)

# if __name__ == '__main__':
#     # Circle parameters
#     circle_center = np.array([2, 2])
#     circle_radius = 1.5
#
#     # Observation point
#     point_outside = np.array([4, 4])
#
#     # Tangent points and their angles
#     tangent_point1, tangent_point2, angle1, angle2 = find_tangent_points(circle_center, circle_radius, point_outside)
#
#     # sample points on the visible arc
#     samples = plot_circle_and_tangent_points(circle_center, circle_radius, point_outside, tangent_point1,
#                                              tangent_point2)
#
#     # Angle for the handle (in degrees)
#     perpendicular_angle_degrees = 30
#
#     angle1_rc = np.arctan2(tangent_point1[1] - circle_center[1], tangent_point1[0] - circle_center[0])
#     angle2_rc = np.arctan2(tangent_point2[1] - circle_center[1], tangent_point2[0] - circle_center[0])
#
#     # Length of handle
#     line_length = 1.5
#
#     if max(np.degrees(angle1_rc), np.degrees(angle2_rc)) > perpendicular_angle_degrees > min(np.degrees(angle1_rc),
#                                                                                              np.degrees(angle2_rc)):
#         # Plot the circle with the perpendicular line segment
#         samples = np.vstack(
#             (samples, sample_handle(circle_center, circle_radius, perpendicular_angle_degrees, line_length)))
#
#     for point in samples:
#         plt.plot(point[0], point[1], 'yo', markersize=5)
#     plt.show()
