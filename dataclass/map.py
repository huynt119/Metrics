from typing import List, Optional
from dataclass.ego_state import EgoState, Point2D, StateSE2

class MapObject:
    """
    MapObject class representing map elements like lanes and connectors
    
    Attributes:
        id: Unique identifier
        _incoming_edges: List of incoming connections 
        _outgoing_edges: List of outgoing connections
    """

    def __init__(self, id: int) -> None:
        self.id = id
        self._incoming_edges: List['MapObject'] = []
        self._outgoing_edges: List['MapObject'] = []

    def incoming_edges(self) -> List['MapObject']:
        """List of map objects connected as incoming edges"""
        return self._incoming_edges

    def outgoing_edges(self) -> List['MapObject']:
        """List of map objects connected as outgoing edges"""
        return self._outgoing_edges

    def baseline_path(self):
        """Baseline path of the map object"""
        #TODO: Implement this method
        pass

class Lane(MapObject):
    """
    Lane class
    """
    #TODO: Implement this class

    def __init__(self, id: int) -> None:
        super().__init__(id)

    def speed_limit_mps(self) -> float:
        """
        Getter function for obtaining the speed limit of the lane from map db.
        :return: Speed limit of the lane.
        """
        #TODO: Implement this method
        pass


class Map:
    """
    Map class.

    - Attributes:
        + database: database of map
    """

    #TODO: Implement this class
    
    def is_in_drivable_area(self, point: Point2D, type: str) -> bool:
        """
        Get all drivable objects in map.
        Check if a point is in the drivable area.
        :param point: point.
        :return: a boolean is_in_drivable_area.
        """
        #TODO: Implement this method
        pass

    def get_distance_to_nearest_map_object(self, point: Point2D, type: str):
        """
        Get all drivable objects in map.
        Get distance to the nearest drivable objects.
        Use def distance in Geometry class (shortest distance between two geometries).
        :param point: point.
        :return: tuple of id and distance.
        """
        #TODO: Implement this method
        pass

    def get_center_objects(self, centers: StateSE2) -> Optional[List[MapObject]]:
        """
        :return: list of lane/lane connector of center ego.
        """
        #TODO: Implement this method
        pass

    def get_corners_objects(self, corners: dict[str, Point2D]) -> dict[str, Optional[List[MapObject]]]:
        """
        :return: list of lane/lane connector of ego's corners.
        """
        #TODO: Implement this method
        pass