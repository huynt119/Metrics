from typing import List, Optional
from dataclass.ego_state import EgoState, Point2D, StateSE2
from dataclass.map import Map, MapObject
from dataclass.history import History

class DrivableAreaCompliance:
    """
    DrivableAreaCompliance class.
    """ 

    def __init__(
        self, 
        max_violation_threshold: float
    ) -> None:
        """
        Initialize DrivableAreaCompliance class.
        :param max_violation_threshold: maximum violation threshold.
        """
        self.max_violation_threshold = max_violation_threshold


    def compute_distance_to_map_objects_list(self, point: Point2D, map_objects: Optional[List[MapObject]]) -> float:
        """
        Compute the min distance to a list of map objects.
        Use def distance in Geometry class (shortest distance between two geometries).
        :param point: point.
        :param
        :param map_objects: list of map objects.
        :return: distance.
        """

        #TODO: Implement this method
        pass
        

    def not_in_drivable_area_with_route_object(self, point: Point2D, map_object: Optional[List[MapObject]], map: Map) -> bool:
        """
        Return a boolean is_in_drivable_area.
        :param point: point.
        :param
        :param map_object: lane/lane connector of that point or empty list.
        :param map: map.
        :return: a boolean is_in_drivable_area.
        """
        return not map_object and not map.is_in_drivable_area(point, type='drivable_area')

    def is_corner_far_from_drivable_area(self, map: Map, center_map_objects: Optional[List[MapObject]], ego_corner: Point2D) -> bool:
        """
        Return a boolean that shows if ego_corner is far from drivable area according to the threshold.
        :param map: map api.
        :param center_map_objects: ego's center route obj in iteration.
        :param ego_corner: one of ego's corners.
        :return: boolean is_corner_far_from_drivable_area.
        """
        if center_map_objects:
            distance = self.compute_distance_to_map_objects_list(ego_corner, center_map_objects)
            if distance < self.max_violation_threshold:
                return False

        id_distance_tuple = map.get_distance_to_nearest_map_object(ego_corner, type='drivable_area')

        return id_distance_tuple[1] is None or id_distance_tuple[1] >= self.max_violation_threshold

    def compute_violation_for_iteration(
        self,
        map: Map, 
        corners: dict[str, Point2D], 
        corners_map_objects: dict[str, Optional[List[MapObject]]], 
        center_map_objects: Optional[List[MapObject]],
        far_from_drivable_area: bool
    ) -> tuple[bool, bool]:
        """
        Compute the violation for a single iteration.
        :param map: Map object representing the map
        :param corners: dictionary of corners where keys are corner names and values are Point2D objects
        :param corners_map_objects: dictionary of map objects for each corner
        :param center_map_objects: list of map objects for the center
        :return: Tuple of two boolean values: (not_in_drivable_area, far_from_drivable_area)
        """
        # Find corners that are outside drivable area
        outside_drivable_area_corners = [
            corner_name
            for corner_name in corners.keys()
            if self.not_in_drivable_area_with_route_object(
                corners[corner_name], 
                corners_map_objects.get(corner_name, []), 
                map
            )
        ]

        # Check if any corner is outside drivable area
        not_in_drivable_area = len(outside_drivable_area_corners) > 0

        # Check if corners outside drivable area are too far
        far_from_drivable_area = far_from_drivable_area or any(
            self.is_corner_far_from_drivable_area(
                map, 
                center_map_objects, 
                corners[corner_name]
            )
            for corner_name in outside_drivable_area_corners
        )

        return (not_in_drivable_area, far_from_drivable_area)


    def calculate_drivable_area_compliance_metric(self, map: Map, history: History):
        """
        Calculate the drivable area compliance metrics.
        :param ego_state: EgoState object representing the ego vehicle
        :param map: Map object representing the mapdrivable_area_compliance
        :param history: History object representing the history of the ego vehicle
        :return: The drivable area compliance metrics
        """
        
        ego_states = history.get_ego_state()
        all_ego_corners = [ego_state.get_all_corners() for ego_state in ego_states]
        center_map_objects = [map.get_center_objects(ego_state.center) for ego_state in ego_states]
        corners_map_objects = [map.get_corners_objects(corner) for corner in all_ego_corners]
        corners_in_drivable_area = []
        far_from_drivable_area = False

        for corners, corners_map_object, center_map_object in zip(all_ego_corners, corners_map_objects, center_map_objects):
            not_in_drivable_area, far_from_drivable_area = self.compute_violation_for_iteration(map, corners, corners_map_object, center_map_object, far_from_drivable_area)
            corners_in_drivable_area.append(float(not not_in_drivable_area))

        metric_score = float(not far_from_drivable_area)

        return metric_score, corners_in_drivable_area, far_from_drivable_area