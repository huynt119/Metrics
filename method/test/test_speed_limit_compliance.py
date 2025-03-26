import unittest
from unittest.mock import Mock, patch
import numpy as np

from dataclass.ego_state import EgoState, StateSE2, DynamicCarState, StateVector2D, Point2D
from dataclass.map import Map, MapObject, Lane
from dataclass.history import History
from method.speed_limit_compliance import SpeedLimitCompliance, SpeedLimitViolationExtractor, GenericViolation

class TestSpeedLimitCompliance(unittest.TestCase):
    def setUp(self):
        self.max_overspeed_value_threshold = 5.0
        self.speed_limit_compliance = SpeedLimitCompliance(self.max_overspeed_value_threshold)
        
        # Mock vehicle parameters
        self.vehicle_parameters = {
            'length': 4.0,
            'width': 2.0,
            'rear_axle_to_center': 1.0
        }

    def create_mock_ego_state(self, speed: float, time_point: int) -> EgoState:
        """Helper method to create a mock ego state"""
        center = StateSE2(0.0, 0.0, 0.0)
        velocity_2d = StateVector2D(speed, 0.0)
        acceleration_2d = StateVector2D(0.0, 0.0)
        dynamic_car_state = DynamicCarState(1.0, velocity_2d, acceleration_2d)
        return EgoState(center, dynamic_car_state, time_point, self.vehicle_parameters)

    def create_mock_lane(self, speed_limit: float) -> Lane:
        """Helper method to create a mock lane"""
        lane = Mock(spec=Lane)
        lane.speed_limit_mps.return_value = speed_limit
        return lane

    def test_perfect_compliance(self):
        """Test when vehicle speed is always under speed limit"""
        # Create mock objects
        map_mock = Mock(spec=Map)
        history_mock = Mock(spec=History)
        
        # Setup test scenario
        ego_states = [
            self.create_mock_ego_state(10.0, 1000000),  # 10 m/s
            self.create_mock_ego_state(12.0, 2000000),  # 12 m/s
        ]
        
        lane = self.create_mock_lane(15.0)  # Speed limit 15 m/s
        map_mock.get_center_objects.return_value = [lane]
        
        history_mock.get_ego_state.return_value = ego_states
        
        # Calculate metric
        result = self.speed_limit_compliance.calculate_speed_limit_compliance_metric(map_mock, history_mock)
        
        # Assert perfect compliance
        self.assertEqual(result, 1.0)

    # def test_speed_violation(self):
    #     """Test when vehicle exceeds speed limit"""
    #     # Create mock objects
    #     map_mock = Mock(spec=Map)
    #     history_mock = Mock(spec=History)
        
    #     # Setup test scenario
    #     ego_states = [
    #         self.create_mock_ego_state(20.0, 1000000),  # 20 m/s
    #         self.create_mock_ego_state(20.0, 2000000),  # 20 m/s
    #     ]
        
    #     lane = self.create_mock_lane(15.0)  # Speed limit 15 m/s
    #     map_mock.get_center_objects.return_value = [lane]
        
    #     history_mock.get_ego_state.return_value = ego_states
        
    #     # Calculate metric
    #     result = self.speed_limit_compliance.calculate_speed_limit_compliance_metric(map_mock, history_mock)
        
    #     # Assert violation (result should be less than 1.0)
    #     self.assertLess(result, 1.0)

    def test_no_speed_limit_data(self):
        """Test when no speed limit data is available"""
        # Create mock objects
        map_mock = Mock(spec=Map)
        history_mock = Mock(spec=History)
        
        # Setup test scenario
        ego_states = [
            self.create_mock_ego_state(10.0, 1000000),
        ]
        
        lane = self.create_mock_lane(None)  # No speed limit data
        map_mock.get_center_objects.return_value = [lane]
        
        history_mock.get_ego_state.return_value = ego_states
        
        # Calculate metric
        result = self.speed_limit_compliance.calculate_speed_limit_compliance_metric(map_mock, history_mock)
        
        # Assert perfect compliance when no speed limit data
        self.assertEqual(result, 1.0)

class TestSpeedLimitViolationExtractor(unittest.TestCase):
    def setUp(self):
        self.vehicle_parameters = {
            'length': 4.0,
            'width': 2.0,
            'rear_axle_to_center': 1.0
        }

    def create_mock_ego_state(self, speed: float, time_point: int) -> EgoState:
        center = StateSE2(0.0, 0.0, 0.0)
        velocity_2d = StateVector2D(speed, 0.0)
        acceleration_2d = StateVector2D(0.0, 0.0)
        dynamic_car_state = DynamicCarState(1.0, velocity_2d, acceleration_2d)
        return EgoState(center, dynamic_car_state, time_point, self.vehicle_parameters)

    def test_violation_detection(self):
        """Test violation detection"""
        ego_states = [
            self.create_mock_ego_state(20.0, 1000000),  # Speeding
            self.create_mock_ego_state(10.0, 2000000),  # Normal speed
        ]
        
        extractor = SpeedLimitViolationExtractor(ego_states)
        
        lane = Mock(spec=Lane)
        lane.speed_limit_mps.return_value = 15.0
        
        violation = extractor.get_speed_limit_violation(
            ego_states[0], 1000000, [lane]
        )
        
        self.assertIsNotNone(violation)
        self.assertEqual(violation.violation_depths[0], 5.0)  # 20 - 15 = 5 m/s over

    def test_no_violation(self):
        """Test when no violation occurs"""
        ego_states = [
            self.create_mock_ego_state(10.0, 1000000),  # Normal speed
        ]
        
        extractor = SpeedLimitViolationExtractor(ego_states)
        
        lane = Mock(spec=Lane)
        lane.speed_limit_mps.return_value = 15.0
        
        violation = extractor.get_speed_limit_violation(
            ego_states[0], 1000000, [lane]
        )
        
        self.assertIsNone(violation)

if __name__ == '__main__':
    unittest.main()
