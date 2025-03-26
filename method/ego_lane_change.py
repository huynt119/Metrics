import numpy as np
import numpy.typing as npt
from typing import List, Set, Optional
from dataclass.ego_state import EgoState, Point2D, StateSE2
from dataclass.map import Map, MapObject, Lane
from dataclass.history import History


class LaneChangeStartRecord:
    """
    Class used to keep track of the timestamp of beginning of a lane change and initial lane or
    initial lane connector(s) if lane change starts in areas annotated as intersection.
    """
    
    def __init__(self, start_timestamp: int, initial_lane: Optional[Set[MapObject]]) -> None:
        """
        Initialize a lane change start record
        :param start_timestamp: Timestamp when lane change started (microseconds)
        :param initial_lane: Set of map objects representing initial lane(s)
        """
        self.start_timestamp = start_timestamp
        self.initial_lane = initial_lane


class LaneChangeData:
    """
    Class used to store lane change data, contains the data on beginning of a lane change, its duration
    in micro seconds, target lane and whether it was successful or not.

    A lane change starts if at the previous timestamps ego was fully within a lane/lane connector (initial lane)
    and at the current timestamp either of the front corners enter another lane different than the initial lane.

    A lane change is complete if all corners of ego enter the same lane/lane connectors.

    A lane change is sucessful if all corners of ego enter a target lane that is different than the initial lane
    and fails if either of the following cases happens:
    1. Ego fully returns to the initial lane
    2. Ego enters nondrivable area before completing the lane change
    3. Scenario ends before completing the lane change.
    """

    def __init__(
        self, 
        start_data: LaneChangeStartRecord,
        duration_us: float,
        final_lane: Optional[Set[MapObject]],
        success: bool
    ) -> None:
        """
        Initialize lane change data
        :param start_data: Record of when lane change started
        :param duration_us: Duration of lane change in microseconds
        :param final_lane: Set of map objects representing final lane(s), None if failed
        :param success: Whether lane change completed successfully
        """
        self.start_data = start_data
        self.duration_us = duration_us
        self.final_lane = final_lane
        self.success = success

    def __str__(self) -> str:
        """String representation of lane change data"""
        return (
            f"LaneChangeData(start_time={self.start_data.start_timestamp}, "
            f"duration={self.duration_us}µs, "
            f"success={self.success})"
        )


def _ego_starts_lane_change(
    initial_lane: Optional[Set[MapObject]], start_timestamp: int
) -> Optional[LaneChangeStartRecord]:
    """
    Opens lane change window and stores the information
    :param initial_lane: Set of common/connected route objects of corners of ego at previous timestamp
    :param start_timestamp: The current timestamp
    :return information on starts of a lane change if exists, otherwise None.
    """
    # We don't consider lane change if in the previous timestamp corners were in different lane/lane connectors
    # or all corners were in nondrivable area
    return LaneChangeStartRecord(start_timestamp, initial_lane) if initial_lane else None


def _ego_ends_lane_change(
    open_lane_change: LaneChangeStartRecord, final_lane: Set[MapObject], end_timestamp: int
) -> LaneChangeData:
    """
    Stores the information if ego ends a lane change
    :param open_lane_change: Record of the currently open lane change
    :param final_lane: Set of common/connected route objects of corners of ego when completing a lane change
    :param end_timestamp: The current timestamp
    :return LaneChangeData.
    """
    # Fail if ego exits the drivable area before completing the lane change, set final_lane as None
    if not final_lane:
        return LaneChangeData(
            open_lane_change, end_timestamp - open_lane_change.start_timestamp, final_lane=None, success=False
        )

    initial_lane = open_lane_change.initial_lane
    initial_lane_ids = {obj.id for obj in initial_lane}  # type: ignore
    initial_lane_out_edge_ids = set(get_outgoing_edges_obj_dict(initial_lane).keys())
    initial_lane_or_out_edge_ids = initial_lane_ids.union(initial_lane_out_edge_ids)
    final_lane_ids = {obj.id for obj in final_lane}

    return LaneChangeData(
        open_lane_change,
        end_timestamp - open_lane_change.start_timestamp,
        final_lane,
        success=False if len(set.intersection(initial_lane_or_out_edge_ids, final_lane_ids)) else True,
    )


def find_lane_changes(
    ego_timestamps: npt.NDArray[np.int32], common_or_connected_route_objs: List[Optional[Set[MapObject]]]
) -> List[LaneChangeData]:
    """
    Extracts the lane changes in the scenario
    :param ego_timestamps: Array of times in time_us
    :param common_or_connected_route_objs: list of common or connected lane/lane connectors of corners
    :return List of lane change data in the scenario.
    """
    lane_changes: List[LaneChangeData] = []
    open_lane_change = None

    if common_or_connected_route_objs[0] is None:
        raise ValueError("Scenario starts with corners in different route objects")

    for prev_ind, curr_obj in enumerate(common_or_connected_route_objs[1:]):
        # check there is no open lane change window
        if open_lane_change is None:
            # Check if current common obj is None (so corners are in different ojects)
            if curr_obj is None:
                open_lane_change = _ego_starts_lane_change(
                    initial_lane=common_or_connected_route_objs[prev_ind], start_timestamp=ego_timestamps[prev_ind + 1]
                )

        else:
            # Check if an open lane change ends and store the data
            if curr_obj is not None:
                lane_change_data = _ego_ends_lane_change(
                    open_lane_change, final_lane=curr_obj, end_timestamp=ego_timestamps[prev_ind + 1]
                )
                lane_changes.append(lane_change_data)
                open_lane_change = None

    # Fail lane change and close interval if the open lane change has not completed during the scenario, set
    # final_lane as None

    print(ego_timestamps[-1], open_lane_change.start_timestamp)
    if open_lane_change:
        lane_changes.append(
            LaneChangeData(
                open_lane_change, ego_timestamps[-1] - open_lane_change.start_timestamp, final_lane=None, success=False
            )
        )

    return lane_changes



def get_outgoing_edges_obj_dict(corner_route_object: List[MapObject]) -> dict[str, MapObject]:
    """
    :param corner_route_object: List of lane/lane connectors
    :return dictionary of id and itscorresponding route object of outgoing edges of a given route object
    """
    return {obj_edge.id: obj_edge for obj in corner_route_object for obj_edge in obj.outgoing_edges()}


def get_incoming_edges_obj_dict(corner_route_object: List[MapObject]) -> dict[str, MapObject]:
    """
    :param corner_route_object: List of lane/lane connectors
    :return dictionary of id and itscorresponding route object of incoming edges of a given route object
    """
    return {obj_edge.id: obj_edge for obj in corner_route_object for obj_edge in obj.incoming_edges()}


def get_common_route_object(
    corners_map_obj_ids: List[Set[str]], obj_id_dict: dict[str, MapObject]
) -> Set[MapObject]:
    """
    Extracts common lane/lane connectors of corners
    :param corners_route_obj_ids: List of ids of route objects of corners of ego
    :param obj_id_dict: dictionary of ids and corresponding route objects
    :return set of common route objects, returns an empty set of no common object is found.
    """
    return {obj_id_dict[id] for id in set.intersection(*corners_map_obj_ids)}

def get_connecting_route_object(
    corners_map_obj_list: List[List[MapObject]],
    corners_map_obj_ids: List[Set[str]],
    obj_id_dict: dict[str, MapObject],
) -> Set[MapObject]:
    """
    Extracts connecting (outgoing or incoming) lane/lane connectors of corners
    :param corners_route_obj_list: List of route objects of corners of ego
    :param corners_route_obj_ids: List of ids of route objects of corners of ego
    :param obj_id_dict: dictionary of ids and corresponding route objects
    :return set of connecting route objects, returns an empty set of no connecting object is found.
    """
    all_corners_connecting_obj_ids = set()

    front_left_route_obj, rear_left_route_obj, rear_right_route_obj, front_right_route_obj = corners_map_obj_list
    (
        front_left_route_obj_ids,
        rear_left_route_obj_ids,
        rear_right_route_obj_ids,
        front_right_route_obj_ids,
    ) = corners_map_obj_ids

    rear_right_route_obj_out_edge_dict = get_outgoing_edges_obj_dict(rear_right_route_obj)
    rear_left_route_obj_out_edge_dict = get_outgoing_edges_obj_dict(rear_left_route_obj)
    # Update dictionary of id: route object
    obj_id_dict = {**obj_id_dict, **rear_right_route_obj_out_edge_dict, **rear_left_route_obj_out_edge_dict}

    # Check if a rear corner is in the same or a connected outgoing lane/lane connectors of the other rear corner
    rear_right_obj_or_outgoing_edge = rear_right_route_obj_ids.union(set(rear_right_route_obj_out_edge_dict.keys()))
    rear_left_in_rear_right_obj_or_outgoing_edge = rear_left_route_obj_ids.intersection(rear_right_obj_or_outgoing_edge)

    rear_left_obj_or_outgoing_edge = rear_left_route_obj_ids.union(set(rear_left_route_obj_out_edge_dict.keys()))
    rear_right_in_rear_left_obj_or_outgoing_edge = rear_right_route_obj_ids.intersection(rear_left_obj_or_outgoing_edge)

    rear_corners_connecting_obj_ids = rear_left_in_rear_right_obj_or_outgoing_edge.union(
        rear_right_in_rear_left_obj_or_outgoing_edge
    )

    if len(rear_corners_connecting_obj_ids) > 0:
        # Check if the right/left front corners are in the same or a connected lane/lane connectors
        # of the opposite (left/right) rear corners
        front_left_route_obj_in_edge_dict = get_incoming_edges_obj_dict(front_left_route_obj)
        front_left_obj_or_incoming_edge = front_left_route_obj_ids.union(set(front_left_route_obj_in_edge_dict.keys()))
        front_left_rear_right_common_obj_ids = front_left_obj_or_incoming_edge.intersection(
            rear_right_obj_or_outgoing_edge
        )

        front_right_route_obj_in_edge_dict = get_incoming_edges_obj_dict(front_right_route_obj)
        front_right_obj_or_incoming_edge = front_right_route_obj_ids.union(
            set(front_right_route_obj_in_edge_dict.keys())
        )
        front_right_rear_left_common_obj_ids = front_right_obj_or_incoming_edge.intersection(
            rear_left_obj_or_outgoing_edge
        )

        all_corners_connecting_obj_ids = {
            obj_id_dict[id]
            for id in set.intersection(front_left_rear_right_common_obj_ids, front_right_rear_left_common_obj_ids)
        }

    return all_corners_connecting_obj_ids



def extract_common_or_connecting_route_objs(
    corner_map_objects: dict[str, Optional[List[MapObject]]],
) -> Optional[Set[MapObject]]:
    """
    Extracts common or connecting (outgoing or incoming) lane/lane connectors of corners
    :param corner_map_objects: Class containing list of lane/lane connectors of each corner
    :return common or connecting lane/lane connectors of corners if exists, else None.
    If all corners are in nondrivable area, returns an empty set.
    """
    corners_map_obj_list = list(corner_map_objects.values())

    not_in_lane_or_laneconn = [
        True if len(corner_map_obj) == 0 else False for corner_map_obj in corners_map_obj_list
    ]

    # Check if all corners are outside lane/lane_connectors
    if np.all(not_in_lane_or_laneconn):
        return set()

    # Check if any corner (not all of them) is outside lane/lane_connectors
    if np.any(not_in_lane_or_laneconn):
        return None

    # Keep a dictionary of ids of lanes/lane connectors to later retrieve them using their ids
    obj_id_dict = {obj.id: obj for corner_map_obj in corners_map_obj_list for obj in corner_map_obj}
    corners_map_obj_ids = [{obj.id for obj in corner_map_obj} for corner_map_obj in corners_map_obj_list]

    # Check if corners are in the same lane/lane connector
    all_corners_common_obj = get_common_route_object(corners_map_obj_ids, obj_id_dict)
    if len(all_corners_common_obj) > 0:
        # Return the set of common lane/lane connectors
        return all_corners_common_obj

    # Check if corners are in a connecting (outgoing or incoming) lane/lane connectors
    all_corners_connecting_obj = get_connecting_route_object(corners_map_obj_list, corners_map_obj_ids, obj_id_dict)
    if len(all_corners_connecting_obj) > 0:
        return all_corners_connecting_obj

    # If no common or conecting route object is found, return None
    return None

def get_common_or_connected_route_objs_of_corners(
    corners_map_objects: List[dict[str, Optional[List[MapObject]]]],
) -> List[Optional[Set[MapObject]]]:
    """
    Returns a list of common or connected lane/lane connectors of corners.
    :param corners_route: List of class conatining list of lane/lane connectors of corners of ego
    :return list of common or connected lane/lane connectors of corners if exist, empty list if all corners are
    in non_drivable area and None if corners are in different lane/lane connectors.
    """
    history_common_or_connecting_route_objs: List[Optional[Set[MapObject]]] = []

    prev_corners_route_obj = corners_map_objects[0]

    corners_common_or_connecting_route_objs = extract_common_or_connecting_route_objs(prev_corners_route_obj)
    history_common_or_connecting_route_objs.append(corners_common_or_connecting_route_objs)

    for curr_corners_route_obj in corners_map_objects[1:]:

        if curr_corners_route_obj != prev_corners_route_obj:

            corners_common_or_connecting_route_objs = extract_common_or_connecting_route_objs(curr_corners_route_obj)

        history_common_or_connecting_route_objs.append(corners_common_or_connecting_route_objs)

        prev_corners_route_obj = curr_corners_route_obj

    return history_common_or_connecting_route_objs

def get_timestamps_in_common_or_connected_route_objs(
    common_or_connected_route_objs: List[Optional[Set[MapObject]]], ego_timestamps: npt.NDArray[np.int32]
) -> List[int]:
    """
    Extract timestamps when ego's corners are in common or connected lane/lane connectors.
    :param common_or_connected_route_objs: list of common or connected lane/lane connectors of corners if exist,
    empty list if all corners are in non_drivable area and None if corners are in different lane/lane connectors
    :param ego_timestamps: Array of times in time_us
    :return List of ego_timestamps where all corners of ego are in common or connected route objects
    """
    return [timestamp for route_obj, timestamp in zip(common_or_connected_route_objs, ego_timestamps) if route_obj]


class EgoLaneChange:
    def __init__(self, max_fail_rate: float):
        self._max_fail_rate = max_fail_rate

    def calculate_ego_lane_change_metric(self, history: History, map: Map):
        """
        Calculate the ego lane change metric.
        :param history: history.
        :param map: map.
        :return: ego lane change metric.
        """
        ego_states = history.get_ego_state()
        ego_centers = [ego_state.center for ego_state in ego_states]
        ego_timestamps = [ego_state.time_point for ego_state in ego_states]
        all_ego_corners = [ego_state.get_all_corners() for ego_state in ego_states]
        center_map_objects = [map.get_center_objects(ego_state.center) for ego_state in ego_states]
        corners_map_objects = [map.get_corners_objects(corner) for corner in all_ego_corners]
        common_or_connected_route_objs = get_common_or_connected_route_objs_of_corners(corners_map_objects)

        # Extract ego timepoints where its corners are in common_or_connected_route objs
        timestamps_in_common_or_connected_route_objs = get_timestamps_in_common_or_connected_route_objs(
            common_or_connected_route_objs, ego_timestamps
        )

        lane_changes = find_lane_changes(ego_timestamps, common_or_connected_route_objs)

        if len(lane_changes) == 0:
            return "No lane changes found"
        else:
            lane_change_durations = [lane_change.duration_us * 1e-6 for lane_change in lane_changes]
            failed_lane_changes = [lane_change for lane_change in lane_changes if not lane_change.success]
            failed_ratio = len(failed_lane_changes) / len(lane_changes)
            fail_rate_below_threshold = True if self._max_fail_rate >= failed_ratio else False
            return lane_changes, lane_change_durations, failed_lane_changes, failed_ratio, fail_rate_below_threshold
