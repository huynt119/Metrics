import numpy as np
from typing import List, Optional
from dataclass.ego_state import EgoState, Point2D, StateSE2
from dataclass.map import Map, MapObject
from dataclass.history import History

class DrivingDirectionCompliance:
    """
    DrivableAreaCompliance class.
    """ 

    def __init__(
        self, 
        driving_direction_compliance_threshold: float = 2,
        driving_direction_violation_threshold: float = 6,
        time_horizon: float = 1, # seconds
    ) -> None:
        self.driving_direction_compliance_threshold = driving_direction_compliance_threshold
        self.driving_direction_violation_threshold = driving_direction_violation_threshold
        self.time_horizon = time_horizon

    def extract_metric(self, centers: List[Point2D], center_map_objects: List[Optional[List[MapObject]]], n_horizon: int) -> List[float]:
        """
        Extract metric from centers and center_map_objects.
        :param centers: centers.
        :param center_map_objects: center map objects.
        :param n_horizon: number of horizon.
        :return: A list of floats including ego's overall movements in the past n_horizon samples.
        """

        progress_along_baseline = []
        distance_to_start = None
        prev_distance_to_start = None
        prev_route_obj_id = None

        # If the first center belongs to a lane/lane_connector store the id in prev_route_obj_id
        if center_map_objects[0]:
            prev_route_obj_id = center_map_objects[0][0].id

        # for each center in the driven_trajectory compute the progress along the baseline of the corresponding lane/lane_connector in driven_route
        for center, center_map_object in zip(centers, center_map_objects):
            # If center isn't assigned a lane/lane_connector, there's no driving direction:
            if not center_map_object:
                progress_along_baseline.append(0.0)
                continue
            # If the lane/lane_conn ego is in hasn't changed since last iteration compute the progress along its baseline
            # by subtracting its current distance to baseline's starting point from its distace in the previous iteration
            if prev_route_obj_id and center_map_object[0].id == prev_route_obj_id:
                distance_to_start = get_distance_of_closest_baseline_point_to_its_start(
                    center_map_object[0].baseline_path(), center
                )
                # If prev_distance_to_start is set, compute the progress by subtracting distance_to_start from it, o.w set it to use in the next iteration
                progress_made = (
                    distance_to_start - prev_distance_to_start
                    if prev_distance_to_start is not None and distance_to_start
                    else 0.0
                )
                progress_along_baseline.append(progress_made)
                prev_distance_to_start = distance_to_start
            else:
                # Reset the parameters when ego first enters a lane/lane-connector
                distance_to_start = None
                prev_distance_to_start = None
                progress_along_baseline.append(0.0)
                prev_route_obj_id = center_map_object[0].id

        # Compute progress over n_horizon last samples for each time point
        progress_over_n_horizon = [
            sum(progress_along_baseline[max(0, ind - n_horizon) : ind + 1])
            for ind, _ in enumerate(progress_along_baseline)
        ]
        return progress_over_n_horizon          


    def calculate_driving_direction_compliance_metric(self, map: Map, history: History) -> float:
        """
        Calculate driving direction compliance metrics.
        :param map: map api.
        :param history: history api.
        :return: driving direction compliance metrics.
        """
        ego_states = history.get_ego_state()
        centers = [ego_state.center.point() for ego_state in ego_states]
        center_map_objects = [map.get_center_objects(ego_state.center) for ego_state in ego_states]
        ego_timestamps = np.array([ego_state.time_point for ego_state in ego_states])
        n_horizon = int(self.time_horizon * 1e6 / np.mean(np.diff(ego_timestamps)))  
        progress_over_interval = self.extract_metric(centers, center_map_objects, n_horizon)

        max_negative_progress_over_interval = abs(min(progress_over_interval))
        if max_negative_progress_over_interval < self.driving_direction_compliance_threshold:
            driving_direction_score = 1.0
        elif max_negative_progress_over_interval < self.driving_direction_violation_threshold:
            driving_direction_score = 0.5
        else:
            driving_direction_score = 0.0


        return driving_direction_score, max_negative_progress_over_interval
    

def get_distance_of_closest_baseline_point_to_its_start(base_line, point: Point2D) -> float:
    """Computes distance of "closest point on the baseline to pose" to the beginning of the baseline
    Use def project in shapely.geometry
    :param base_line: A baseline path
    :param point: An ego center
    :return: distance to start.
    """
    #TODO: Implement this method

    """
    Visual Example
    Baseline:     A----B----C----D----E
                (start)|
                       | (shortest)
                       |
                    pose (P)

    Distance = Length of arc A->B
    where B is the projection of P onto the baseline
    """

    # ex: Original code: return float(base_line.linestring.project(Point(*pose)))
