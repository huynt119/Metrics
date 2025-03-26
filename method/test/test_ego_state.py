import unittest
import numpy as np
from dataclass.ego_state import (
    Point2D, StateSE2, StateVector2D, DynamicCarState, EgoState,
    translate, translate_longitudinally, translate_longitudinally_and_laterally,
    get_velocity_shifted, get_acceleration_shifted, approximate_derivatives,
    phase_unwrap, extract_ego_heading, extract_ego_acceleration,
    extract_ego_yaw_rate, extract_ego_jerk
)

class TestPoint2D(unittest.TestCase):
    def test_point2d_initialization(self):
        point = Point2D(1.0, 2.0)
        self.assertEqual(point.x, 1.0)
        self.assertEqual(point.y, 2.0)

class TestStateSE2(unittest.TestCase):
    def test_statese2_initialization(self):
        state = StateSE2(1.0, 2.0, 0.5)
        self.assertEqual(state.x, 1.0)
        self.assertEqual(state.y, 2.0)
        self.assertEqual(state.heading, 0.5)

    def test_point_method(self):
        state = StateSE2(1.0, 2.0, 0.5)
        point = state.point()
        self.assertIsInstance(point, Point2D)
        self.assertEqual(point.x, 1.0)
        self.assertEqual(point.y, 2.0)

class TestStateVector2D(unittest.TestCase):
    def test_statevector2d_initialization(self):
        vector = StateVector2D(3.0, 4.0)
        self.assertEqual(vector.x, 3.0)
        self.assertEqual(vector.y, 4.0)
        np.testing.assert_array_equal(vector.array, np.array([3.0, 4.0]))

    def test_magnitude(self):
        vector = StateVector2D(3.0, 4.0)
        self.assertEqual(vector.magnitude(), 5.0)

class TestDynamicCarState(unittest.TestCase):
    def setUp(self):
        self.velocity = StateVector2D(1.0, 0.0)
        self.acceleration = StateVector2D(0.1, 0.0)
        self.car_state = DynamicCarState(
            rear_axle_to_center_dist=2.0,
            rear_axle_velocity_2d=self.velocity,
            rear_axle_acceleration_2d=self.acceleration,
            angular_velocity=0.1,
            angular_acceleration=0.05,
            tire_steering_rate=0.02
        )

    def test_initialization(self):
        self.assertEqual(self.car_state._rear_axle_to_center_dist, 2.0)
        self.assertEqual(self.car_state.angular_velocity(), 0.1)
        self.assertEqual(self.car_state.angular_acceleration(), 0.05)
        self.assertEqual(self.car_state.tire_steering_rate(), 0.02)

    def test_speed(self):
        self.assertEqual(self.car_state.speed(), 1.0)

    def test_acceleration(self):
        self.assertEqual(self.car_state.acceleration(), 0.1)

class TestEgoState(unittest.TestCase):
    def setUp(self):
        self.center = StateSE2(0.0, 0.0, 0.0)
        self.velocity = StateVector2D(1.0, 0.0)
        self.acceleration = StateVector2D(0.1, 0.0)
        self.dynamic_car_state = DynamicCarState(
            rear_axle_to_center_dist=2.0,
            rear_axle_velocity_2d=self.velocity,
            rear_axle_acceleration_2d=self.acceleration
        )
        self.vehicle_parameters = {
            'length': 4.0,
            'width': 2.0,
            'rear_axle_to_center': 1.0
        }
        self.ego_state = EgoState(
            center=self.center,
            dynamic_car_state=self.dynamic_car_state,
            time_point=1000000,  # 1 second in microseconds
            vehicle_parameters=self.vehicle_parameters
        )

    def test_initialization(self):
        self.assertEqual(self.ego_state.time_point, 1000000)
        self.assertEqual(len(self.ego_state.corners), 4)

    def test_get_corner(self):
        corner = self.ego_state.get_corner('front_left')
        self.assertIsInstance(corner, Point2D)
        
        with self.assertRaises(KeyError):
            self.ego_state.get_corner('invalid_corner')

    def test_rear_axle(self):
        rear_axle = self.ego_state.rear_axle()
        self.assertIsInstance(rear_axle, StateSE2)
        self.assertEqual(rear_axle.x, -1.0)  # Based on rear_axle_to_center = 1.0

class TestHelperFunctions(unittest.TestCase):
    def test_translate(self):
        pose = StateSE2(1.0, 1.0, 0.0)
        translation = np.array([2.0, 3.0])
        result = translate(pose, translation)
        self.assertEqual(result.x, 3.0)
        self.assertEqual(result.y, 4.0)
        self.assertEqual(result.heading, 0.0)

    def test_phase_unwrap(self):
        headings = np.array([-np.pi, np.pi, -np.pi], dtype=np.float32)
        unwrapped = phase_unwrap(headings)
        self.assertTrue(np.all(np.abs(np.diff(unwrapped)) <= np.pi))

if __name__ == '__main__':
    unittest.main()
