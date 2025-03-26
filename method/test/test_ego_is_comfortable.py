import unittest
import numpy as np
from unittest.mock import Mock, patch
from method.ego_is_comfortable import EgoIsComfortable
from dataclass.ego_state import EgoState, StateSE2, Point2D, DynamicCarState, StateVector2D

class TestEgoIsComfortable(unittest.TestCase):
    def setUp(self):
        # Set up test parameters
        self.min_lon_accel = -4.0  # m/s^2
        self.max_lon_accel = 2.0   # m/s^2
        self.max_abs_lat_accel = 2.0  # m/s^2
        self.max_abs_yaw_accel = 1.0  # rad/s^2
        self.max_abs_yaw_rate = 0.5   # rad/s
        self.max_abs_lon_jerk = 2.5   # m/s^3
        self.max_abs_mag_jerk = 3.0   # m/s^3
        
        self.comfort_checker = EgoIsComfortable(
            min_lon_accel=self.min_lon_accel,
            max_lon_accel=self.max_lon_accel,
            max_abs_lat_accel=self.max_abs_lat_accel,
            max_abs_yaw_accel=self.max_abs_yaw_accel,
            max_abs_yaw_rate=self.max_abs_yaw_rate,
            max_abs_lon_jerk=self.max_abs_lon_jerk,
            max_abs_mag_jerk=self.max_abs_mag_jerk
        )

    def test_check_threshold(self):
        """Test the _check_threshold method with different scenarios"""
        # Test with both min and max thresholds
        values = np.array([-1.0, 0.0, 1.0])
        self.assertTrue(self.comfort_checker._check_threshold(values, max_threshold=2.0, min_threshold=-2.0))
        self.assertFalse(self.comfort_checker._check_threshold(values, max_threshold=0.5, min_threshold=-0.5))

        # Test with only max threshold
        self.assertTrue(self.comfort_checker._check_threshold(values, max_threshold=2.0))
        self.assertFalse(self.comfort_checker._check_threshold(values, max_threshold=0.5))

        # Test with absolute values
        values = np.array([-2.0, 1.0, -1.5])
        self.assertTrue(self.comfort_checker._check_threshold(values, max_threshold=2.0, use_absolute=True))
        self.assertFalse(self.comfort_checker._check_threshold(values, max_threshold=1.0, use_absolute=True))

    def create_mock_ego_states(self, num_states=10):
        """Helper method to create mock ego states"""
        ego_states = []
        vehicle_parameters = {
            'length': 4.0,
            'width': 2.0,
            'rear_axle_to_center': 1.0
        }
        
        for i in range(num_states):
            # Create basic components for EgoState
            center = StateSE2(x=float(i), y=float(i), heading=0.1 * i)
            velocity = StateVector2D(1.0, 0.0)
            acceleration = StateVector2D(0.5, 0.2)
            dynamic_state = DynamicCarState(
                rear_axle_to_center_dist=1.0,
                rear_axle_velocity_2d=velocity,
                rear_axle_acceleration_2d=acceleration
            )
            
            # Create EgoState
            ego_state = EgoState(
                center=center,
                dynamic_car_state=dynamic_state,
                time_point=i * 1e6,  # microseconds
                vehicle_parameters=vehicle_parameters
            )
            ego_states.append(ego_state)
            
        return ego_states

    def test_calculate_ego_is_comfortable_metric_comfortable_case(self):
        """Test when all metrics are within comfortable ranges"""
        # Create mock History and Map
        mock_history = Mock()
        mock_map = Mock()
        
        # Set up mock ego states with comfortable values
        mock_history.get_ego_states.return_value = self.create_mock_ego_states()
        
        # Test the comfort metric
        result = self.comfort_checker.calculate_ego_is_comfortable_metric(mock_history, mock_map)
        self.assertEqual(result, 1.0)

    @patch('method.ego_is_comfortable.extract_ego_acceleration')
    def test_calculate_ego_is_comfortable_metric_uncomfortable_acceleration(self, mock_extract_accel):
        """Test when acceleration exceeds comfortable ranges"""
        mock_history = Mock()
        mock_map = Mock()
        mock_history.get_ego_states.return_value = self.create_mock_ego_states()
        
        # Mock uncomfortable acceleration
        mock_extract_accel.return_value = np.array([5.0, -5.0])  # Outside comfort range
        
        result = self.comfort_checker.calculate_ego_is_comfortable_metric(mock_history, mock_map)
        self.assertEqual(result, 0.0)

    @patch('method.ego_is_comfortable.extract_ego_yaw_rate')
    def test_calculate_ego_is_comfortable_metric_uncomfortable_yaw_rate(self, mock_extract_yaw):
        """Test when yaw rate exceeds comfortable ranges"""
        mock_history = Mock()
        mock_map = Mock()
        mock_history.get_ego_states.return_value = self.create_mock_ego_states()
        
        # Mock uncomfortable yaw rate
        mock_extract_yaw.return_value = np.array([1.0, -1.0])  # Outside comfort range
        
        result = self.comfort_checker.calculate_ego_is_comfortable_metric(mock_history, mock_map)
        self.assertEqual(result, 0.0)

if __name__ == '__main__':
    unittest.main()
