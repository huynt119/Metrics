import unittest
from unittest.mock import Mock, patch
import numpy as np
from typing import List, Set, Optional

from method.ego_lane_change import (
    LaneChangeStartRecord,
    LaneChangeData,
    find_lane_changes,
    extract_common_or_connecting_route_objs,
    get_common_or_connected_route_objs_of_corners,
    _ego_starts_lane_change,
    _ego_ends_lane_change,
    EgoLaneChange
)
from dataclass.map import MapObject, Map
from dataclass.ego_state import EgoState, Point2D, StateSE2
from dataclass.history import History

class TestEgoLaneChange(unittest.TestCase):
    def setUp(self):
        # Create mock map objects
        self.map_obj1 = Mock(spec=MapObject)
        self.map_obj1.id = "lane1"
        self.map_obj1.outgoing_edges.return_value = []
        
        self.map_obj2 = Mock(spec=MapObject)
        self.map_obj2.id = "lane2"
        self.map_obj2.outgoing_edges.return_value = []

        # Set up timestamps array
        self.timestamps = np.array([1000000, 2000000, 3000000], dtype=np.int32)

    def test_lane_change_start_record(self):
        """Test LaneChangeStartRecord initialization"""
        initial_lane = {self.map_obj1}
        record = LaneChangeStartRecord(1000000, initial_lane)
        
        self.assertEqual(record.start_timestamp, 1000000)
        self.assertEqual(record.initial_lane, initial_lane)

    def test_lane_change_data(self):
        """Test LaneChangeData initialization and string representation"""
        start_record = LaneChangeStartRecord(1000000, {self.map_obj1})
        lane_change = LaneChangeData(start_record, 2000000, {self.map_obj2}, True)
        
        self.assertEqual(lane_change.duration_us, 2000000)
        self.assertEqual(lane_change.final_lane, {self.map_obj2})
        self.assertTrue(lane_change.success)
        self.assertIn("LaneChangeData", str(lane_change))

    def test_ego_starts_lane_change(self):
        """Test _ego_starts_lane_change function"""
        initial_lane = {self.map_obj1}
        start_timestamp = 1000000
        
        # Test with valid initial lane
        result = _ego_starts_lane_change(initial_lane, start_timestamp)
        self.assertIsInstance(result, LaneChangeStartRecord)
        self.assertEqual(result.start_timestamp, start_timestamp)
        self.assertEqual(result.initial_lane, initial_lane)
        
        # Test with None initial lane
        result = _ego_starts_lane_change(None, start_timestamp)
        self.assertIsNone(result)

    def test_ego_ends_lane_change(self):
        """Test _ego_ends_lane_change function"""
        start_record = LaneChangeStartRecord(1000000, {self.map_obj1})
        
        # Test successful lane change
        final_lane = {self.map_obj2}
        result = _ego_ends_lane_change(start_record, final_lane, 2000000)
        self.assertTrue(result.success)
        self.assertEqual(result.duration_us, 1000000)
        
        # Test failed lane change (empty final lane)
        result = _ego_ends_lane_change(start_record, set(), 2000000)
        self.assertFalse(result.success)
        self.assertIsNone(result.final_lane)

    # def test_find_lane_changes(self):
    #     """Test find_lane_changes function"""
    #     # Create a sequence of route objects representing a lane change
    #     route_objs = [
    #         {self.map_obj1},  # Initial lane
    #         None,             # During lane change
    #         {self.map_obj2}   # Final lane
    #     ]
        
    #     lane_changes = find_lane_changes(self.timestamps, route_objs)
        
    #     self.assertEqual(len(lane_changes), 1)
    #     self.assertTrue(lane_changes[0].success)
    #     self.assertEqual(lane_changes[0].duration_us, 2000000)

    # def test_ego_lane_change_metric(self):
    #     """Test EgoLaneChange class and metric calculation"""
    #     ego_lane_change = EgoLaneChange(max_fail_rate=0.5)
        
    #     # Mock History and Map objects
    #     history = Mock(spec=History)
    #     map_obj = Mock(spec=Map)
        
    #     # Mock ego states
    #     ego_states = [Mock(spec=EgoState) for _ in range(3)]
    #     for state in ego_states:
    #         state.center = Mock(spec=StateSE2)
    #         state.get_all_corners.return_value = {
    #             'front_left': Mock(spec=Point2D),
    #             'front_right': Mock(spec=Point2D),
    #             'rear_left': Mock(spec=Point2D),
    #             'rear_right': Mock(spec=Point2D)
    #         }
        
    #     history.get_ego_state.return_value = ego_states
        
    #     # Set up map objects for the test
    #     map_obj.get_corners_objects.side_effect = [
    #         {'front_left': [self.map_obj1], 'front_right': [self.map_obj1],
    #          'rear_left': [self.map_obj1], 'rear_right': [self.map_obj1]},
    #         {'front_left': [self.map_obj1, self.map_obj2], 'front_right': [self.map_obj1, self.map_obj2],
    #          'rear_left': [self.map_obj1], 'rear_right': [self.map_obj1]},
    #         {'front_left': [self.map_obj2], 'front_right': [self.map_obj2],
    #          'rear_left': [self.map_obj2], 'rear_right': [self.map_obj2]}
    #     ]
        
    #     result = ego_lane_change.calculate_ego_lane_change_metric(history, map_obj)
        
    #     # Check if result contains expected components
    #     self.assertIsInstance(result, tuple)
    #     lane_changes, durations, failed_changes, fail_ratio, below_threshold = result
    #     self.assertIsInstance(lane_changes, list)
    #     self.assertIsInstance(durations, list)
    #     self.assertIsInstance(failed_changes, list)
    #     self.assertIsInstance(fail_ratio, float)
    #     self.assertIsInstance(below_threshold, bool)

if __name__ == '__main__':
    unittest.main()
