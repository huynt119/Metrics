import numpy as np
from typing import List, Optional
from dataclass.ego_state import EgoState, Point2D, StateSE2
from dataclass.map import Map, MapObject, Lane
from dataclass.history import History


class GenericViolation:
    """Class used to keep track of violations, contains the depth of violation as well as their timestamp."""
    def __init__(
        self, 
        time_point: int,
        violation_depths: List[float]
    ) -> None:
        self.time_point = time_point
        self.violation_depths = violation_depths
    


class SpeedLimitViolationExtractor:
    """Class to extract speed limit violations."""

    def __init__(
        self, 
        ego_states: List[EgoState],
    ) -> None:
        """
        Initializes the SpeedLimitViolationExtractor class
        :param ego_states: Ego's states.
        """
        self.ego_states = ego_states
        self.open_violation: Optional[List[GenericViolation]] = None
        self.violation_depths: List[float] = []
        self.violations: List[GenericViolation] = None

    def extract_metric(self, center_map_objects: List[Optional[List[MapObject]]]) -> None:
        """Extracts the drivable area violations from the history of Ego poses."""
        time_point = None
        for ego_state, curr_ego_obj in zip(self.ego_states, center_map_objects):
            time_point = ego_state.time_point

            # If no lane or lane connector is associated with pose (such as when ego is
            # outside the drivable area), we won't consider speed limit violation
            if not curr_ego_obj:
                violation = None
            else:
                violation = self.get_speed_limit_violation(ego_state, time_point, curr_ego_obj) 

            if violation:
                if not self.open_violation:
                    self.start_violation(violation)
                else:
                    self.update_violation(violation)
                self.violation_depths.append(violation.violation_depths[0])
            else:
                self.violation_depths.append(0)
                if self.open_violation:
                    self.end_violation(time_point)
        # End all violations
        if time_point and self.open_violation:
            self.end_violation(time_point)

    def start_violation(self, violation: GenericViolation) -> None:
        """
        Opens the violation window of the given IDs, as they now starting to violate the metric
        :param violation: The current violation.
        """
        self.open_violation = violation

    def update_violation(self, violation: GenericViolation) -> None:
        """
        Updates the violation if the maximum depth of violation is greater than the current maximum
        :param violation: The current violation.
        """
        assert isinstance(self.open_violation, GenericViolation), 'There is no open violation, cannot update it!'
        self.open_violation.violation_depths.extend(violation.violation_depths)

    def end_violation(self, time_point: int) -> None:
        """
        Closes the violation window, as Ego re-enters the non-violating regime
        :param timestamp: The current timestamp
        """
        assert isinstance(self.open_violation, GenericViolation), 'There is no open violation, cannot end it!'

        self.open_violation = None

    def get_speed_limit_violation(
        self,
        ego_state: EgoState, timestamp: int, ego_lane_or_laneconnector: Optional[List[MapObject]]
    ) -> Optional[GenericViolation]:
        """
        Computes by how much ego is exceeding the speed limit
        :param ego_state: The current state of Ego
        :param timestamp: The current timestamp
        :return: By how much ego is exceeding the speed limit, none if not violation is present or unable to find
        the speed limit.
        """
        if isinstance(ego_lane_or_laneconnector[0], Lane):
            assert len(ego_lane_or_laneconnector) == 1, 'Ego should can assigned to one lane only'
            speed_limits = [ego_lane_or_laneconnector[0].speed_limit_mps()]
        else:
            speed_limits = []
            for map_obj in ego_lane_or_laneconnector:
                edges = map_obj.outgoing_edges() + map_obj.incoming_edges()
                speed_limits.extend([lane.speed_limit_mps() for lane in edges])

        # new map can potentially return None if the GPKG does not contain speed limit data,
        # make sure speed limits exist
        if all(speed_limits):
            max_speed_limit = max(speed_limits)
            exceeding_speed = ego_state.dynamic_car_state.speed() - max_speed_limit
            return GenericViolation(timestamp, violation_depths=[exceeding_speed]) if exceeding_speed > 0 else None

        return None




class SpeedLimitCompliance:
    """
    SpeedLimitCompliance class.
    """ 

    def __init__(self, max_overspeed_value_threshold: float) -> None:
        self.max_overspeed_value_threshold = max_overspeed_value_threshold

    def calculate_speed_limit_compliance_metric(self, map: Map, history: History) -> float:
        """
        Calculate speed limit compliance.
        :param ego_state: ego state.
        :param map: map.
        :return: speed limit compliance.
        """
        ego_states = history.get_ego_state()
        center_map_objects = [map.get_center_objects(ego_state.center) for ego_state in ego_states]
        extractor = SpeedLimitViolationExtractor(ego_states=ego_states)
        extractor.extract_metric(center_map_objects)
        time_stamps = [ego_state.time_point for ego_state in ego_states]
        if not extractor.violations:
            return 1.0
        else:
            dt_in_sec = np.mean(np.diff(time_stamps)) * 1e-6
            scenario_duration_in_sec = (time_stamps[-1] - time_stamps[0]) * 1e-6
            # Adding a small tolerance to handle cases where max_overspeed_value_threshold is specified as 0
            max_overspeed_value_threshold = max(self.max_overspeed_value_threshold, 1e-3)
            violation_loss = (
                np.sum(extractor.violation_depths) * dt_in_sec / (max_overspeed_value_threshold * scenario_duration_in_sec)
            )
            return float(max(0.0, 1.0 - violation_loss))
