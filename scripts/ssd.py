import numpy as np


def point_to_line_segment_distance(point, line_start, line_end):
    line_vec = line_end - line_start
    point_vec = point - line_start
    line_length = np.linalg.norm(line_vec)
    line_unit_vec = line_vec / line_length

    projection = np.dot(point_vec, line_unit_vec)
    if projection < 0:
        return np.linalg.norm(point - line_start)
    if projection > line_length:
        return np.linalg.norm(point - line_end)
    perpendicular_distance = np.linalg.norm(point_vec - projection * line_unit_vec)
    return perpendicular_distance


def compute_distance_to_circle(point, circle_center, circle_radius):
    distance = np.linalg.norm(point - circle_center) - circle_radius
    return abs(distance)


def compute_combined_rmse(observed_points, mug_center, mug_radius, handle_angle, handle_length):
    line_start = mug_center + mug_radius * np.array([np.cos(handle_angle), np.sin(handle_angle)])
    line_end = line_start + handle_length * np.array([np.cos(handle_angle), np.sin(handle_angle)])

    squared_distances = []
    for point in observed_points.T:
        line_distance = point_to_line_segment_distance(point, line_start, line_end)
        circle_distance = compute_distance_to_circle(point, mug_center, mug_radius)
        combined_distance = min(line_distance, circle_distance)
        squared_distances.append(combined_distance ** 2)

    mean_squared_distance = np.mean(squared_distances)
    rmse = np.sqrt(mean_squared_distance)
    return rmse


# if __name__ == '__main__':
#     # Example observed points (replace with your actual observed points)
#     observed_points = np.array([[1, 2], [3, 4], [5, 6], [7, 8], [9, 10]])
#
#     # Example endpoints of the line segment (replace with your actual endpoints)
#     line_start = np.array([2, 2])
#     line_end = np.array([8, 8])
#
#     # Example circle parameters (replace with your actual center and radius)
#     circle_center = np.array([5, 5])
#     circle_radius = 3
#
#     # Compute combined RMSE for the plane with line segment and circle
#     combined_rmse = compute_combined_rmse(observed_points, line_start, line_end, circle_center, circle_radius)
#
#     print("Combined RMSE:", combined_rmse)
