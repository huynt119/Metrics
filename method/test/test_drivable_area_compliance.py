import unittest
from unittest.mock import Mock, patch
from dataclass.ego_state import Point2D, StateSE2, EgoState
from dataclass.map import Map, MapObject
from dataclass.history import History
from method.drivable_area_compliance import DrivableAreaCompliance

class TestDrivableAreaCompliance(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures"""
        self.max_violation_threshold = 0.5
        self.dac = DrivableAreaCompliance(self.max_violation_threshold)
        
        # Mock map objects
        self.mock_map_object = Mock(spec=MapObject)
        self.mock_map = Mock(spec=Map)
        self.mock_history = Mock(spec=History)

    def test_compute_distance_to_map_objects_list_empty(self):
        """Test compute_distance_to_map_objects_list with empty map objects"""
        point = Point2D(0.0, 0.0)
        distance = self.dac.compute_distance_to_map_objects_list(point, [])
        self.assertIsNone(distance)

    def test_not_in_drivable_area_with_route_object(self):
        """Test not_in_drivable_area_with_route_object method"""
        point = Point2D(0.0, 0.0)
        
        # Case 1: With map objects and in drivable area
        self.mock_map.is_in_drivable_area.return_value = True
        result = self.dac.not_in_drivable_area_with_route_object(
            point, [self.mock_map_object], self.mock_map
        )
        self.assertFalse(result)

        # Case 2: No map objects and not in drivable area
        self.mock_map.is_in_drivable_area.return_value = False
        result = self.dac.not_in_drivable_area_with_route_object(
            point, [], self.mock_map
        )
        self.assertTrue(result)

    # def test_is_corner_far_from_drivable_area(self):
    #     """Test is_corner_far_from_drivable_area method"""
    #     corner = Point2D(1.0, 1.0)
        
    #     # Case 1: Within threshold
    #     self.mock_map.get_distance_to_nearest_map_object.return_value = (1, 0.3)
    #     result = self.dac.is_corner_far_from_drivable_area(
    #         self.mock_map, [self.mock_map_object], corner
    #     )
    #     self.assertFalse(result)

    #     # Case 2: Beyond threshold
    #     self.mock_map.get_distance_to_nearest_map_object.return_value = (1, 0.6)
    #     result = self.dac.is_corner_far_from_drivable_area(
    #         self.mock_map, [self.mock_map_object], corner
    #     )
    #     self.assertTrue(result)

    def test_compute_violation_for_iteration(self):
        """Test compute_violation_for_iteration method"""
        corners = {
            'front_left': Point2D(1.0, 1.0),
            'front_right': Point2D(1.0, -1.0),
            'rear_left': Point2D(-1.0, 1.0),
            'rear_right': Point2D(-1.0, -1.0)
        }
        corners_map_objects = {
            'front_left': [self.mock_map_object],
            'front_right': [self.mock_map_object],
            'rear_left': [self.mock_map_object],
            'rear_right': [self.mock_map_object]
        }
        
        # Mock map behavior
        self.mock_map.is_in_drivable_area.return_value = True
        self.mock_map.get_distance_to_nearest_map_object.return_value = (1, 0.3)

        not_in_drivable, far_from_drivable = self.dac.compute_violation_for_iteration(
            self.mock_map,
            corners,
            corners_map_objects,
            [self.mock_map_object],
            False
        )
        
        self.assertFalse(not_in_drivable)
        self.assertFalse(far_from_drivable)

    def test_calculate_drivable_area_compliance_metric(self):
        """Test calculate_drivable_area_compliance_metric method"""
        # Mock ego states
        mock_ego_state = Mock(spec=EgoState)
        mock_ego_state.get_all_corners.return_value = {
            'front_left': Point2D(1.0, 1.0),
            'front_right': Point2D(1.0, -1.0),
            'rear_left': Point2D(-1.0, 1.0),
            'rear_right': Point2D(-1.0, -1.0)
        }
        mock_ego_state.center = StateSE2(0.0, 0.0, 0.0)
        
        # Set up mock returns
        self.mock_history.get_ego_state.return_value = [mock_ego_state]
        self.mock_map.get_center_objects.return_value = [self.mock_map_object]
        self.mock_map.get_corners_objects.return_value = {
            'front_left': [self.mock_map_object],
            'front_right': [self.mock_map_object],
            'rear_left': [self.mock_map_object],
            'rear_right': [self.mock_map_object]
        }
        self.mock_map.is_in_drivable_area.return_value = True
        self.mock_map.get_distance_to_nearest_map_object.return_value = (1, 0.3)

        metric_score, corners_in_drivable, far_from_drivable = (
            self.dac.calculate_drivable_area_compliance_metric(
                self.mock_map, self.mock_history
            )
        )

        self.assertEqual(metric_score, 1.0)
        self.assertEqual(corners_in_drivable, [1.0])
        self.assertFalse(far_from_drivable)

if __name__ == '__main__':
    unittest.main()
