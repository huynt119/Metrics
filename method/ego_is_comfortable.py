import numpy as np
from typing import List
from dataclass.ego_state import EgoState, Point2D, StateSE2, extract_ego_acceleration, extract_ego_yaw_rate, extract_ego_jerk
from dataclass.map import Map, MapObject, Lane
from dataclass.history import History

class EgoIsComfortable:
    def __init__(self, min_lon_accel: float, max_lon_accel: float, max_abs_lat_accel: float, 
                 max_abs_yaw_accel:float, max_abs_yaw_rate = float, 
                 max_abs_lon_jerk = float, max_abs_mag_jerk = float) -> None:
        """
        Initializes the EgoIsComfortable class
        :param min_lon_accel: Minimum threshold to define if the lon acceleration is within bound
        :param max_lon_accel: Maximum threshold to define if the lon acceleration is within bound.
        """
        self.min_lon_accel = min_lon_accel
        self.max_lon_accel = max_lon_accel
        self.max_abs_lat_accel = max_abs_lat_accel
        self.max_abs_yaw_accel = max_abs_yaw_accel
        self.max_abs_yaw_rate = max_abs_yaw_rate
        self.max_abs_lon_jerk = max_abs_lon_jerk
        self.max_abs_mag_jerk = max_abs_mag_jerk

    def _check_threshold(self, values: np.ndarray, max_threshold: float = None, 
                        min_threshold: float = None, use_absolute: bool = False) -> bool:
        """
        Check if values are within thresholds
        :param values: Array of values to check
        :param max_threshold: Maximum threshold value (None if no upper bound)
        :param min_threshold: Minimum threshold value (None if no lower bound)
        :param use_absolute: If True, compare absolute values against threshold
        :return: True if all values are within thresholds
        """
        check_values = np.abs(values) if use_absolute else values
        
        if max_threshold is not None and min_threshold is not None:
            return np.all((check_values >= min_threshold) & (check_values <= max_threshold))
        elif max_threshold is not None:
            return np.all(check_values <= max_threshold)
        elif min_threshold is not None:
            return np.all(check_values >= min_threshold)
        return True

    def calculate_ego_is_comfortable_metric(self, history: History, map: Map) -> bool:
        """
        Compute comfortability based on all thresholds
        :param history: History from a simulation engine
        :param map: Map instance
        :return True if within all thresholds otherwise false
        """
        ego_states = history.get_ego_states()
        
        # Extract metrics
        ego_lon_accels = extract_ego_acceleration(ego_states, acceleration_coordinate='x')
        ego_lat_accels = extract_ego_acceleration(ego_states, acceleration_coordinate='y')
        yaw_rates = extract_ego_yaw_rate(ego_states)
        yaw_accels = extract_ego_yaw_rate(ego_states, deriv_order=2, poly_order=3)
        ego_lon_jerks = extract_ego_jerk(ego_states, acceleration_coordinate='x')
        ego_jerks = extract_ego_jerk(ego_states, acceleration_coordinate='magnitude')

        # Check all comfort criteria
        checks = [
            self._check_threshold(ego_lon_accels, self.max_lon_accel, self.min_lon_accel),
            self._check_threshold(ego_lat_accels, self.max_abs_lat_accel, use_absolute=True),
            self._check_threshold(yaw_rates, self.max_abs_yaw_rate, use_absolute=True),
            self._check_threshold(yaw_accels, self.max_abs_yaw_accel, use_absolute=True),
            self._check_threshold(ego_lon_jerks, self.max_abs_lon_jerk, use_absolute=True),
            self._check_threshold(ego_jerks, self.max_abs_mag_jerk, use_absolute=True)
        ]

        if all(checks):
            return 1.0
        return 0.0


