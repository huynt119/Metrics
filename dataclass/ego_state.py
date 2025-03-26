import numpy as np
import numpy.typing as npt
import math
from typing import List
from scipy.signal import savgol_filter

class Point2D:
    """Class to represents 2D points."""

    def __init__(self, x: float, y: float) -> None:
        """
        Initialize Point2D class.
        :param x: x-coordinate.
        :param y: y-coordinate.
        """
        self.x = x
        self.y = y

class StateSE2(Point2D):
    """
    StateSE2 class.
    """

    def __init__(self, x: float, y: float, heading: float) -> None:
        """
        Initialize StateSE2 class.
        :param x: x-coordinate.
        :param
        :param y: y-coordinate.
        :param heading: heading.
        """
        super().__init__(x, y)
        self.heading = heading

    def point(self) -> Point2D:
        """
        Gets a point from the StateSE2
        :return: Point with x and y from StateSE2
        """
        return Point2D(self.x, self.y)
    
class StateVector2D:
    """Representation of vector in 2d."""

    def __init__(self, x: float, y: float):
        """
        Create StateVector2D object
        :param x: float direction
        :param y: float direction
        """
        self.x = x  # x-axis in the vector.
        self.y = y  # y-axis in the vector.
        self.array: npt.NDArray[np.float64] = np.array([self.x, self.y], dtype=np.float64)

    
    def magnitude(self) -> float:
        """
        :return: magnitude of vector
        """
        return float(np.hypot(self.x, self.y))
    
class DynamicCarState:
    """Contains the various dynamic attributes of ego."""

    def __init__(
        self,
        rear_axle_to_center_dist: float,
        rear_axle_velocity_2d: StateVector2D,
        rear_axle_acceleration_2d: StateVector2D,
        angular_velocity: float = 0.0,
        angular_acceleration: float = 0.0,
        tire_steering_rate: float = 0.0,
    ):
        """
        :param rear_axle_to_center_dist:[m]  Distance (positive) from rear axle to the geometrical center of ego
        :param rear_axle_velocity_2d: [m/s]Velocity vector at the rear axle
        :param rear_axle_acceleration_2d: [m/s^2] Acceleration vector at the rear axle
        :param angular_velocity: [rad/s] Angular velocity of ego
        :param angular_acceleration: [rad/s^2] Angular acceleration of ego
        :param tire_steering_rate: [rad/s] Tire steering rate of ego
        """
        self._rear_axle_to_center_dist = rear_axle_to_center_dist
        self._angular_velocity = angular_velocity
        self._angular_acceleration = angular_acceleration
        self._rear_axle_velocity_2d = rear_axle_velocity_2d
        self._rear_axle_acceleration_2d = rear_axle_acceleration_2d
        self._tire_steering_rate = tire_steering_rate

    def rear_axle_velocity_2d(self) -> StateVector2D:
        """
        Returns the vectorial velocity at the middle of the rear axle.
        :return: StateVector2D Containing the velocity at the rear axle
        """
        return self._rear_axle_velocity_2d

    def rear_axle_acceleration_2d(self) -> StateVector2D:
        """
        Returns the vectorial acceleration at the middle of the rear axle.
        :return: StateVector2D Containing the acceleration at the rear axle
        """
        return self._rear_axle_acceleration_2d

    def center_velocity_2d(self) -> StateVector2D:
        """
        Returns the vectorial velocity at the geometrical center of Ego.
        :return: StateVector2D Containing the velocity at the geometrical center of Ego
        """
        displacement = StateVector2D(self._rear_axle_to_center_dist, 0.0)
        return get_velocity_shifted(displacement, self.rear_axle_velocity_2d(), self.angular_velocity())

    def center_acceleration_2d(self) -> StateVector2D:
        """
        Returns the vectorial acceleration at the geometrical center of Ego.
        :return: StateVector2D Containing the acceleration at the geometrical center of Ego
        """
        displacement = StateVector2D(self._rear_axle_to_center_dist, 0.0)
        return get_acceleration_shifted(
            displacement, self.rear_axle_acceleration_2d(), self.angular_velocity(), self.angular_acceleration()
        )

    def angular_velocity(self) -> float:
        """
        Getter for the angular velocity of ego.
        :return: [rad/s] Angular velocity
        """
        return self._angular_velocity

    def angular_acceleration(self) -> float:
        """
        Getter for the angular acceleration of ego.
        :return: [rad/s^2] Angular acceleration
        """
        return self._angular_acceleration

    def tire_steering_rate(self) -> float:
        """
        Getter for the tire steering rate of ego.
        :return: [rad/s] Tire steering rate
        """
        return self._tire_steering_rate

    def speed(self) -> float:
        """
        Magnitude of the speed of the center of ego.
        :return: [m/s] 1D speed
        """
        return float(self._rear_axle_velocity_2d.magnitude())

    def acceleration(self) -> float:
        """
        Magnitude of the acceleration of the center of ego.
        :return: [m/s^2] 1D acceleration
        """
        return float(self._rear_axle_acceleration_2d.magnitude())

    def __eq__(self, other: object) -> bool:
        """
        Compare two instances whether they are numerically close
        :param other: object
        :return: true if the classes are almost equal
        """
        if not isinstance(other, DynamicCarState):
            # Return NotImplemented in case the classes do not match
            return NotImplemented

        return (
            self.rear_axle_velocity_2d() == other.rear_axle_velocity_2d()
            and self.rear_axle_acceleration_2d() == other.rear_axle_acceleration_2d()
            and math.isclose(self._angular_acceleration, other._angular_acceleration)
            and math.isclose(self._angular_velocity, other._angular_velocity)
            and math.isclose(self._rear_axle_to_center_dist, other._rear_axle_to_center_dist)
            and math.isclose(self._tire_steering_rate, other._tire_steering_rate)
        )

    def __repr__(self) -> str:
        """Repr magic method"""
        return (
            f"Rear Axle| velocity: {self.rear_axle_velocity_2d()}, acceleration: {self.rear_axle_acceleration_2d()}\n"
            f"Center   | velocity: {self.center_velocity_2d()}, acceleration: {self.center_acceleration_2d()}\n"
            f"angular velocity: {self.angular_velocity()}, angular acceleration: {self.angular_acceleration()}\n"
            f"rear_axle_to_center_dist: {self._rear_axle_to_center_dist} \n"
            f"_tire_steering_rate: {self._tire_steering_rate} \n"
        )
    
class EgoState:
    """
    EgoState class.
    """

    center: StateSE2 
    dynamic_car_state: DynamicCarState
    time_point: int # microseconds
    corners: dict[str, Point2D]
    vehicle_parameters: dict[str, float]
    __slots__ = ("center", "dynamic_car_state", 
                 "time_point", "corners", "vehicle_parameters"
                )

    def __init__(self, center: StateSE2, dynamic_car_state: DynamicCarState, time_point: float, 
                 vehicle_parameters: dict[str, float]) -> None:
        """
        Initialize EgoState class.
        :param center: center of the ego
        :param dynamic_car_state: dynamic state of the ego
        :param time_point: time_point of the ego
        :param vehicle_parameters: dictionary containing vehicle parameters
        """
        self.center = center
        self.dynamic_car_state = dynamic_car_state
        self.time_point = time_point
        self.vehicle_parameters = vehicle_parameters
        self.corners = self.get_all_corners()

    def get_all_corners(self) -> dict[str, Point2D]:
        """
        Get the corners of the ego.
        :return: dictionary of corners where keys are corner names and values are Point2D objects
        """
        front_left = translate_longitudinally_and_laterally(self.center, self.vehicle_parameters['length'] / 2, self.vehicle_parameters['width'] / 2).point()
        front_right = translate_longitudinally_and_laterally(self.center, self.vehicle_parameters['length'] / 2, -self.vehicle_parameters['width'] / 2).point()
        rear_left = translate_longitudinally_and_laterally(self.center, -self.vehicle_parameters['length'] / 2, self.vehicle_parameters['width'] / 2).point()
        rear_right = translate_longitudinally_and_laterally(self.center, -self.vehicle_parameters['length'] / 2, -self.vehicle_parameters['width'] / 2).point()
        corners = {
            'front_left': front_left,
            'front_right': front_right,
            'rear_left': rear_left,
            'rear_right': rear_right
        }

        return corners

    def get_corner(self, corner_name: str) -> Point2D:
        """
        Get a specific corner of the ego.
        :param corner_name: name of the corner ('front_left', 'front_right', 'rear_left', 'rear_right')
        :return: Point2D object representing the corner position
        :raises: KeyError if corner_name is invalid
        """
        if corner_name not in self.corners:
            raise KeyError(f"Invalid corner name: {corner_name}. Valid names are: {list(self.corners.keys())}")
        return self.corners[corner_name]
    
    def rear_axle(self) -> StateSE2:
        """
        Getter for the pose at the middle of the rear axle
        :return: SE2 Pose of the rear axle.
        """
        return translate_longitudinally(self.center, -float(self.vehicle_parameters['rear_axle_to_center']))
    
    
def translate(pose: StateSE2, translation: npt.NDArray[np.float64]) -> StateSE2:
    """ "
    Applies a 2D translation
    :param pose: The pose to be transformed
    :param translation: The translation to be applied
    :return: The translated pose
    """
    assert translation.shape == (2,) or translation.shape == (2, 1)
    return StateSE2(pose.x + translation[0], pose.y + translation[1], pose.heading)

def translate_longitudinally(pose: StateSE2, distance: float) -> StateSE2:
    """
    Translate an SE2 pose longitudinally (along heading direction)
    :param pose: SE2 pose to be translated
    :param distance: [m] distance by which point (x, y, heading) should be translated longitudinally
    :return translated se2
    """
    translation: npt.NDArray[np.float64] = np.array([distance * np.cos(pose.heading), distance * np.sin(pose.heading)])
    return translate(pose, translation)

def translate_longitudinally_and_laterally(pose: StateSE2, lon: float, lat: float) -> StateSE2:
    """
    Translate the position component of an SE2 pose longitudinally and laterally
    :param pose: SE2 pose to be translated
    :param lon: [m] distance by which a point should be translated in longitudinal direction
    :param lat: [m] distance by which a point should be translated in lateral direction
    :return Point2D translated position
    """
    half_pi = np.pi / 2.0
    translation: npt.NDArray[np.float64] = np.array(
        [
            (lat * np.cos(pose.heading + half_pi)) + (lon * np.cos(pose.heading)),
            (lat * np.sin(pose.heading + half_pi)) + (lon * np.sin(pose.heading)),
        ]
    )
    return translate(pose, translation)

def get_velocity_shifted(
    displacement: StateVector2D, ref_velocity: StateVector2D, ref_angular_vel: float
) -> StateVector2D:
    """
    Computes the velocity at a query point on the same planar rigid body as a reference point.
    :param displacement: [m] The displacement vector from the reference to the query point
    :param ref_velocity: [m/s] The velocity vector at the reference point
    :param ref_angular_vel: [rad/s] The angular velocity of the body around the vertical axis
    :return: [m/s] The velocity vector at the given displacement.
    """
    # From cross product of velocity transfer formula in 2D
    velocity_shift_term: npt.NDArray[np.float64] = np.array(
        [-displacement.y * ref_angular_vel, displacement.x * ref_angular_vel]
    )
    return StateVector2D(*(ref_velocity.array + velocity_shift_term))


def get_acceleration_shifted(
    displacement: StateVector2D, ref_accel: StateVector2D, ref_angular_vel: float, ref_angular_accel: float
) -> StateVector2D:
    """
    Computes the acceleration at a query point on the same planar rigid body as a reference point.
    :param displacement: [m] The displacement vector from the reference to the query point
    :param ref_accel: [m/s^2] The acceleration vector at the reference point
    :param ref_angular_vel: [rad/s] The angular velocity of the body around the vertical axis
    :param ref_angular_accel: [rad/s^2] The angular acceleration of the body around the vertical axis
    :return: [m/s^2] The acceleration vector at the given displacement.
    """
    centripetal_acceleration_term = displacement.array * ref_angular_vel**2
    angular_acceleration_term = displacement.array * ref_angular_accel

    return StateVector2D(*(ref_accel.array + centripetal_acceleration_term + angular_acceleration_term))

def approximate_derivatives(
    y: npt.NDArray[np.float32],
    x: npt.NDArray[np.float32],
    window_length: int = 5,
    poly_order: int = 2,
    deriv_order: int = 1,
    axis: int = -1,
) -> npt.NDArray[np.float32]:
    """
    Given two equal-length sequences y and x, compute an approximation to the n-th
    derivative of some function interpolating the (x, y) data points, and return its
    values at the x's.  We assume the x's are increasing and equally-spaced.
    :param y: The dependent variable (say of length n)
    :param x: The independent variable (must have the same length n).  Must be strictly
        increasing and equally-spaced.
    :param window_length: The order (default 5) of the Savitsky-Golay filter used.
        (Ignored if the x's are not equally-spaced.)  Must be odd and at least 3
    :param poly_order: The degree (default 2) of the filter polynomial used.  Must
        be less than the window_length
    :param deriv_order: The order of derivative to compute (default 1)
    :param axis: The axis of the array x along which the filter is to be applied. Default is -1.
    :return Derivatives.
    """
    window_length = min(window_length, len(x))

    if not (poly_order < window_length):
        raise ValueError(f'{poly_order} < {window_length} does not hold!')

    dx = np.diff(x)
    if not (dx > 0).all():
        raise RuntimeError('dx is not monotonically increasing!')

    dx = dx.mean()
    derivative: npt.NDArray[np.float32] = savgol_filter(
        y, polyorder=poly_order, window_length=window_length, deriv=deriv_order, delta=dx, axis=axis
    )
    return derivative

def phase_unwrap(headings: npt.NDArray[np.float32]) -> npt.NDArray[np.float32]:
    """
    Returns an array of heading angles equal mod 2 pi to the input heading angles,
    and such that the difference between successive output angles is less than or
    equal to pi radians in absolute value
    :param headings: An array of headings (radians)
    :return The phase-unwrapped equivalent headings.
    """
    # There are some jumps in the heading (e.g. from -np.pi to +np.pi) which causes approximation of yaw to be very large.
    # We want unwrapped[j] = headings[j] - 2*pi*adjustments[j] for some integer-valued adjustments making the absolute value of
    # unwrapped[j+1] - unwrapped[j] at most pi:
    # -pi <= headings[j+1] - headings[j] - 2*pi*(adjustments[j+1] - adjustments[j]) <= pi
    # -1/2 <= (headings[j+1] - headings[j])/(2*pi) - (adjustments[j+1] - adjustments[j]) <= 1/2
    # So adjustments[j+1] - adjustments[j] = round((headings[j+1] - headings[j]) / (2*pi)).
    two_pi = 2.0 * np.pi
    adjustments = np.zeros_like(headings)
    adjustments[1:] = np.cumsum(np.round(np.diff(headings) / two_pi))
    unwrapped = headings - two_pi * adjustments
    return unwrapped

def extract_ego_heading(ego_states: List[EgoState]) -> npt.NDArray[np.float32]:
    """
    Extract yaw headings of ego pose in simulation history
    :param ego_states: A list of ego states
    :return An array of ego pose yaw heading.
    """
    heading: npt.NDArray[np.float32] = np.array([ego_state.rear_axle().heading for ego_state in ego_states])
    return heading

def extract_ego_acceleration(
    ego_states: List[EgoState],
    acceleration_coordinate: str,
    decimals: int = 8,
    poly_order: int = 2,
    window_length: int = 8,
) -> npt.NDArray[np.float32]:
    """
    Extract acceleration of ego pose in simulation history
    :param ego_states: A list of ego states
    :param acceleration_coordinate: 'x', 'y', or 'magnitude'
    :param decimals: Decimal precision
    :return An array of ego pose acceleration.
    """
    if acceleration_coordinate == 'x':
        acceleration: npt.NDArray[np.float32] = np.asarray(
            [ego_state.dynamic_car_state.center_acceleration_2d().x for ego_state in ego_states]
        )
    elif acceleration_coordinate == 'y':
        acceleration = np.asarray([ego_state.dynamic_car_state.center_acceleration_2d().y for ego_state in ego_states])
    elif acceleration_coordinate == 'magnitude':
        acceleration = np.array([ego_state.dynamic_car_state.acceleration() for ego_state in ego_states])
    else:
        raise ValueError(
            f'acceleration_coordinate option: {acceleration_coordinate} not available. '
            f'Available options are: x, y or magnitude'
        )
    acceleration = savgol_filter(
        acceleration, polyorder=poly_order, window_length=min(window_length, len(acceleration))
    )
    acceleration = np.round(acceleration, decimals=decimals)
    return acceleration

def extract_ego_yaw_rate(
    ego_states: List[EgoState],
    deriv_order: int = 1,
    poly_order: int = 2,
    decimals: int = 8,
    window_length: int = 15,
) -> npt.NDArray[np.float32]:
    """
    Extract ego rates
    :param ego_states: A list of ego states
    :param poly_order: The degree (default 2) of the filter polynomial used.  Must
        be less than the window_length
    :param deriv_order: The order of derivative to compute (default 1)
    :param decimals: Decimal precision
    :return An array of ego yaw rates.
    """
    ego_headings = extract_ego_heading(ego_states)
    ego_timestamps = np.array([ego_state.time_point for ego_state in ego_states])
    ego_yaw_rate = approximate_derivatives(
        phase_unwrap(ego_headings), ego_timestamps / 1e6, deriv_order=deriv_order, poly_order=poly_order
    )  # convert to seconds
    ego_yaw_rate = np.round(ego_yaw_rate, decimals=decimals)
    return ego_yaw_rate

def extract_ego_jerk(
    ego_states: List[EgoState],
    acceleration_coordinate: str,
    decimals: int = 8,
    deriv_order: int = 1,
    poly_order: int = 2,
    window_length: int = 15,
) -> npt.NDArray[np.float32]:
    """
    Extract jerk of ego pose in simulation history
    :param ego_states: A list of ego states
    :param acceleration_coordinate: x, y or 'magnitude' in acceleration
    :param decimals: Decimal precision
    :return An array of valid ego pose jerk and timestamps.
    """
    time_points = np.array([ego_state.time_point for ego_state in ego_states])
    ego_acceleration = extract_ego_acceleration(ego_states=ego_states, acceleration_coordinate=acceleration_coordinate)

    jerk = approximate_derivatives(
        ego_acceleration,
        time_points / 1e6,
        deriv_order=deriv_order,
        poly_order=poly_order,
        window_length=min(window_length, len(ego_acceleration)),
    )  # Convert to seconds
    jerk = np.round(jerk, decimals=decimals)

    return jerk

