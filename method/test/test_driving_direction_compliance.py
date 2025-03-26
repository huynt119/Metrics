import unittest
from unittest.mock import Mock, patch
import numpy as np
from dataclass.ego_state import Point2D, StateSE2, EgoState
from dataclass.map import Map, MapObject
from dataclass.history import History
from method.driving_direction_compliance import DrivingDirectionCompliance

class TestDrivingDirectionCompliance(unittest.TestCase):
    def setUp(self):
        self.metric = DrivingDirectionCompliance(
            driving_direction_compliance_threshold=2,
            driving_direction_violation_threshold=6,
            time_horizon=1
        )
        
    def test_init(self):
        """Test initialization of DrivingDirectionCompliance class"""
        self.assertEqual(self.metric.driving_direction_compliance_threshold, 2)
        self.assertEqual(self.metric.driving_direction_violation_threshold, 6)
        self.assertEqual(self.metric.time_horizon, 1)

    def test_extract_metric_no_map_objects(self):
        """Test extract_metric when no map objects are present"""
        centers = [Point2D(0, 0), Point2D(1, 1)]
        center_map_objects = [None, None]
        n_horizon = 2
        result = self.metric.extract_metric(centers, center_map_objects, n_horizon)
        self.assertEqual(result, [0.0, 0.0])

    @patch('method.driving_direction_compliance.get_distance_of_closest_baseline_point_to_its_start')
    def test_extract_metric_with_map_objects(self, mock_get_distance):
        """Test extract_metric with valid map objects"""
        # Mock MapObject
        map_obj = Mock(spec=MapObject)
        map_obj.id = 1
        map_obj.baseline_path.return_value = "mock_baseline"
        
        # Setup test data
        centers = [Point2D(0, 0), Point2D(1, 1)]
        center_map_objects = [[map_obj], [map_obj]]
        n_horizon = 2
        
        # Mock distance calculations
        mock_get_distance.side_effect = [1.0, 2.0]
        
        result = self.metric.extract_metric(centers, center_map_objects, n_horizon)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0], 0.0)  # First point has no previous distance
        self.assertEqual(result[1], 1.0)  # Second point: 2.0 - 1.0 = 1.0

    def test_calculate_driving_direction_compliance_metric(self):
        """Test driving direction compliance calculation"""
        # Mock Map and History
        mock_map = Mock(spec=Map)
        mock_history = Mock(spec=History)
        
        # Mock MapObject
        map_obj = Mock(spec=MapObject)
        map_obj.id = 1
        map_obj.baseline_path.return_value = "mock_baseline"

        # Create test ego states
        ego_states = [
            Mock(spec=EgoState, center=StateSE2(0, 0, 0), time_point=0),
            Mock(spec=EgoState, center=StateSE2(1, 1, 0), time_point=1000000)  # 1 second later
        ]
        
        # Setup mock returns
        mock_history.get_ego_state.return_value = ego_states
        mock_map.get_center_objects.return_value = [map_obj]
        
        # Test compliant case
        with patch('method.driving_direction_compliance.get_distance_of_closest_baseline_point_to_its_start') as mock_dist:
            mock_dist.side_effect = [1.0, 2.0]  # Forward progress
            score, progress = self.metric.calculate_driving_direction_compliance_metric(mock_map, mock_history)
            self.assertEqual(score, 1.0)  # Should be compliant
            
        # Test violation case
        with patch('method.driving_direction_compliance.get_distance_of_closest_baseline_point_to_its_start') as mock_dist:
            mock_dist.side_effect = [10.0, 2.0]  # Backward progress
            score, progress = self.metric.calculate_driving_direction_compliance_metric(mock_map, mock_history)
            self.assertEqual(score, 0.0)  # Should be violation

if __name__ == '__main__':
    unittest.main()
