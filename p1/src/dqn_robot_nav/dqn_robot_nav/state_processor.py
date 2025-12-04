import numpy as np
import math

class StateProcessor:
    """
    Procesa el LiDAR y la información del robot para producir un estado compacto
    para el DQN. Salida final: vector de tamaño 12.
    - 10 bins LiDAR normalizados
    - distancia normalizada al objetivo
    - ángulo relativo al objetivo normalizado
    """
    def __init__(self, n_lidar_bins=10, max_lidar_range=3.5):
        self.n_lidar_bins = n_lidar_bins
        self.max_lidar_range = max_lidar_range

    def get_state(self, lidar_ranges, robot_position, goal_position, robot_yaw):

        data = np.array(lidar_ranges, dtype=float)
        data = np.where(np.isinf(data), self.max_lidar_range, data)
        data = np.nan_to_num(data, nan=self.max_lidar_range)

        bin_size = len(data) // self.n_lidar_bins
        lidar_bins = []

        for i in range(self.n_lidar_bins):
            start = i * bin_size
            end = start + bin_size
            minimum = np.min(data[start:end])
            lidar_bins.append(minimum / self.max_lidar_range)

        lidar_bins = np.clip(lidar_bins, 0.0, 1.0)

        rx, ry = robot_position
        gx, gy = goal_position

        dx = gx - rx
        dy = gy - ry

        dist = math.sqrt(dx*dx + dy*dy)
        dist_norm = min(dist / 5.0, 1.0)

        goal_theta = math.atan2(dy, dx)
        angle_error = goal_theta - robot_yaw
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

        angle_norm = angle_error / np.pi

        state = np.concatenate([
            lidar_bins,
            np.array([dist_norm, angle_norm])
        ])

        return state.astype(np.float32)
