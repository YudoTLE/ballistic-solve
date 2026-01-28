from collections.abc import Callable, Sequence
import enum
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray


class Environment:
    """
    Environmental conditions for ballistic simulation.

    Encapsulates atmospheric and gravitational conditions affecting projectile motion.
    """

    def __init__(self, gravity: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], air_density: Callable[[float], float], wind_velocity: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]) -> None:
        """Wind velocity vector (m/s)"""

    @property
    def gravity(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Gravitational acceleration vector in m/s²"""

    @property
    def air_density(self) -> Callable[[float], float]:
        """Air density function ρ(altitude) in kg/m³"""

    @property
    def wind_velocity(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Wind velocity vector in m/s"""

    @staticmethod
    def vacuum() -> Environment:
        """
        Create a vacuum environment (no air, no wind).

        Uses standard gravity but zero air density. Useful for testing.
        """

    @staticmethod
    def standard() -> Environment:
        """
        Create standard Earth atmosphere at sea level.

        Uses standard gravity (9.81 m/s²) and exponential atmosphere model.
        """

    @staticmethod
    def standard_with_random_wind(seed: int | None = None) -> Environment:
        """Optional random seed for reproducibility"""

    @staticmethod
    def standard_with_random_strong_wind(seed: int | None = None) -> Environment:
        """Optional random seed for reproducibility"""

    @staticmethod
    def standard_with_random_extreme_wind(seed: int | None = None) -> Environment:
        """Optional random seed for reproducibility"""

    @staticmethod
    def standard_with_random_catastrophic_wind(seed: int | None = None) -> Environment:
        """Optional random seed for reproducibility"""

class Projectile:
    """
    Physical properties of a projectile for ballistic simulation.

    Encapsulates the mass, cross-sectional area, and drag coefficient
    needed to compute aerodynamic forces during flight.
    """

    def __init__(self, mass: float = 1.0, area: float = 1.0, drag_coefficient: float = 0.47) -> None:
        """Dimensionless drag coefficient (default: 0.47, sphere)"""

    @property
    def mass(self) -> float:
        """Mass of the projectile in kilograms"""

    @property
    def area(self) -> float:
        """Cross-sectional area in square meters (perpendicular to velocity)"""

    @property
    def drag_coefficient(self) -> float:
        """
        Dimensionless drag coefficient. Typical values:
        - Sphere: ~0.47
        - Streamlined body: ~0.04
        - Flat plate: ~1.28
        """

class Platform:
    """
    Launch platform properties for ballistic simulation.

    Represents the firing platform (e.g., ground position, moving vehicle, aircraft)
    including its position, velocity, and the muzzle velocity of the weapon system.
    The projectile's initial velocity is the vector sum of platform velocity and
    muzzle velocity in the launch direction.
    """

    def __init__(self, position: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], velocity: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], muzzle_velocity: float) -> None:
        """Scalar muzzle velocity in m/s (projectile speed relative to platform)"""

    @property
    def position(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Position of the launch platform in 3D space (meters)"""

    @property
    def velocity(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """
        Velocity of the platform itself (m/s). For stationary platforms, this is zero.
        """

    @property
    def muzzle_velocity(self) -> float:
        """
        Muzzle velocity magnitude (m/s). Speed of projectile relative to platform.
        """

class Trajectory:
    """
    Container for a computed ballistic trajectory.

    Stores the position history and corresponding time stamps of a projectile
    as computed by the trajectory integration. Positions and times are recorded
    at each adaptive integration step.
    """

    def __init__(self, positions: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], times: Sequence[float]) -> None:
        """Vector of time stamps (must have same length as positions)"""

    def size(self) -> int:
        """Get the number of recorded points in the trajectory."""

    def empty(self) -> bool:
        """Check if the trajectory is empty."""

    @property
    def positions(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]]:
        """Sequence of 3D positions along the trajectory"""

    @property
    def times(self) -> list[float]:
        """Corresponding time stamps for each position (seconds)"""

class IntegrationOptions:
    """
    Configuration for the adaptive ODE integrator.

    Controls the Dormand-Prince (DOPRI5) Runge-Kutta adaptive stepper.
    """

    def __init__(self, max_step: float = 1000.0, first_step: float = 0.001, atol: float = 1e-06, rtol: float = 0.001) -> None:
        """Relative error tolerance"""

    @property
    def max_step(self) -> float:
        """Maximum step size (seconds)"""

    @property
    def first_step(self) -> float:
        """Initial step size (seconds)"""

    @property
    def atol(self) -> float:
        """Absolute error tolerance"""

    @property
    def rtol(self) -> float:
        """Relative error tolerance"""

class TargetingSolution:
    """
    Complete targeting solution for engaging a target.

    Contains launch angles, direction vector, and time of flight.
    """

    @property
    def angles(self) -> tuple[float, float]:
        """Launch angles as (azimuth, elevation) in radians"""

    @property
    def direction(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
        """Launch direction (unit vector)"""

    @property
    def time(self) -> float:
        """Time of flight (seconds)"""

class AngleSearchStrategy(enum.Enum):
    """Strategy for nested angle optimization."""

    AzimuthThenElevation = 0
    """Optimize azimuth in inner loop, elevation in outer loop"""

    ElevationThenAzimuth = 1
    """Optimize elevation in inner loop, azimuth in outer loop"""

class TargetingOptions:
    """
    Configuration for the ballistic solver.

    Controls search strategy and iteration limits for finding firing solutions.
    """

    def __init__(self, angle_search_strategy: AngleSearchStrategy = AngleSearchStrategy.AzimuthThenElevation, azimuth_max_iter: int = 16, elevation_max_iter: int = 16, time_max_iter: int = 16, h: float = 0.001, time_scan_step: float = 0.5) -> None:
        """Time step for coarse scanning in time search (seconds)"""

    @property
    def angle_search_strategy(self) -> AngleSearchStrategy:
        """Strategy for nested angle search"""

    @property
    def azimuth_max_iter(self) -> int:
        """Maximum iterations for azimuth angle search"""

    @property
    def elevation_max_iter(self) -> int:
        """Maximum iterations for elevation angle search"""

    @property
    def time_max_iter(self) -> int:
        """Maximum iterations for time-of-flight search"""

    @property
    def h(self) -> float:
        """Step size for numerical differentiation"""

    @property
    def time_scan_step(self) -> float:
        """Time step for coarse scanning in time search (seconds)"""

def to_direction(angles: tuple[float, float]) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
    """
    Convert spherical coordinates to 3D direction vector.

    Args:
        angles: Pair of (azimuth, elevation) angles in radians

    Returns:
        Unit direction vector
    """

def to_angles(direction: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]) -> tuple[float, float]:
    """
    Convert 3D direction vector to spherical coordinates.

    Args:
        direction: Direction vector (need not be normalized)

    Returns:
        Tuple of (azimuth, elevation) in radians
    """

@overload
def compute_trajectory(environment: Environment, projectile: Projectile, platform: Platform, direction: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], time: float, integration_options: IntegrationOptions = ...) -> Trajectory:
    """
    Compute complete ballistic trajectory with recorded time steps.

    Simulates projectile motion under gravity, drag, and wind using adaptive
    Runge-Kutta integration. Records position at each integration step.

    Args:
        environment: Environmental conditions
        projectile: Projectile properties
        platform: Launch platform
        direction: Unit vector indicating launch direction
        time: Simulation time (seconds)
        integration_options: Integration parameters

    Returns:
        Complete trajectory with positions and times
    """

@overload
def compute_trajectory(environment: Environment, projectile: Projectile, platform: Platform, angles: tuple[float, float], time: float, integration_options: IntegrationOptions = ...) -> Trajectory:
    """
    Compute complete ballistic trajectory from launch angles.

    Convenience overload that accepts (azimuth, elevation) angles.

    Args:
        environment: Environmental conditions
        projectile: Projectile properties
        platform: Launch platform
        angles: Pair of (azimuth, elevation) angles in radians
        time: Simulation time (seconds)
        integration_options: Integration parameters

    Returns:
        Complete trajectory with positions and times
    """

@overload
def compute_point(environment: Environment, projectile: Projectile, platform: Platform, direction: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], time: float, integration_options: IntegrationOptions = ...) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
    """
    Compute final position only (no trajectory recording).

    More efficient than compute_trajectory when only the endpoint is needed.

    Args:
        environment: Environmental conditions
        projectile: Projectile properties
        platform: Launch platform
        direction: Unit vector indicating launch direction
        time: Simulation time (seconds)
        integration_options: Integration parameters

    Returns:
        Final position after given simulation time
    """

@overload
def compute_point(environment: Environment, projectile: Projectile, platform: Platform, angles: tuple[float, float], time: float, integration_options: IntegrationOptions = ...) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
    """
    Compute final position only from launch angles.

    Args:
        environment: Environmental conditions
        projectile: Projectile properties
        platform: Launch platform
        angles: Pair of (azimuth, elevation) angles in radians
        time: Simulation time (seconds)
        integration_options: Integration parameters

    Returns:
        Final position after given simulation time
    """

def find_best_azimuth_angle(environment: Environment, projectile: Projectile, platform: Platform, target_position: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], elevation_angle: float, time: float, integration_options: IntegrationOptions = ..., targeting_options: TargetingOptions = ...) -> float:
    """
    Find best azimuth angle for a given elevation and time.

    Args:
        environment: Environmental conditions
        projectile: Projectile properties
        platform: Launch platform
        target_position: Target position
        elevation_angle: Fixed elevation angle (radians)
        time: Flight time (seconds)
        integration_options: Integration parameters
        targeting_options: Targeting configuration

    Returns:
        Best azimuth angle (radians)
    """

def find_best_elevation_angle(environment: Environment, projectile: Projectile, platform: Platform, target_position: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], azimuth_angle: float, time: float, integration_options: IntegrationOptions = ..., targeting_options: TargetingOptions = ...) -> float:
    """
    Find best elevation angle for a given azimuth and time.

    Args:
        environment: Environmental conditions
        projectile: Projectile properties
        platform: Launch platform
        target_position: Target position
        azimuth_angle: Fixed azimuth angle (radians)
        time: Flight time (seconds)
        integration_options: Integration parameters
        targeting_options: Targeting configuration

    Returns:
        Best elevation angle (radians)
    """

def find_best_angles(environment: Environment, projectile: Projectile, platform: Platform, target_position: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], time: float, integration_options: IntegrationOptions = ..., targeting_options: TargetingOptions = ...) -> tuple[float, float]:
    """
    Find best launch angles (azimuth, elevation) for a given time.

    Uses nested search based on the configured angle search strategy.
    Minimizes squared distance to target.

    Args:
        environment: Environmental conditions
        projectile: Projectile properties
        platform: Launch platform
        target_position: Target position
        time: Flight time (seconds)
        integration_options: Integration parameters
        targeting_options: Targeting configuration

    Returns:
        Tuple of (azimuth, elevation) in radians
    """

def find_best_direction(environment: Environment, projectile: Projectile, platform: Platform, target_position: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], time: float, integration_options: IntegrationOptions = ..., targeting_options: TargetingOptions = ...) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]:
    """
    Find best launch direction for a given time.

    Convenience function that returns a direction vector instead of angles.

    Args:
        environment: Environmental conditions
        projectile: Projectile properties
        platform: Launch platform
        target_position: Target position
        time: Flight time (seconds)
        integration_options: Integration parameters
        targeting_options: Targeting configuration

    Returns:
        Best launch direction (unit vector)
    """

def find_earliest_time(environment: Environment, projectile: Projectile, platform: Platform, target_position: Callable[[float], Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], time_range: tuple[float, float], integration_options: IntegrationOptions = ..., targeting_options: TargetingOptions = ...) -> float | None:
    """
    Find earliest time when target can be intercepted.

    Searches forward through the time range to find the first moment when
    the projectile can reach the (possibly moving) target. Uses coarse
    scanning followed by root refinement.

    Args:
        environment: Environmental conditions
        projectile: Projectile properties
        platform: Launch platform
        target_position: Function returning target position at given time
        time_range: Tuple of (min_time, max_time) in seconds
        integration_options: Integration parameters
        targeting_options: Targeting configuration

    Returns:
        Earliest intercept time in seconds, or None if no solution exists
    """

def find_latest_time(environment: Environment, projectile: Projectile, platform: Platform, target_position: Callable[[float], Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], time_range: tuple[float, float], integration_options: IntegrationOptions = ..., targeting_options: TargetingOptions = ...) -> float | None:
    """
    Find latest time when target can be intercepted.

    Searches backward through the time range to find the last moment when
    the projectile can reach the (possibly moving) target. Uses coarse
    scanning followed by root refinement.

    Args:
        environment: Environmental conditions
        projectile: Projectile properties
        platform: Launch platform
        target_position: Function returning target position at given time
        time_range: Tuple of (min_time, max_time) in seconds
        integration_options: Integration parameters
        targeting_options: Targeting configuration

    Returns:
        Latest intercept time in seconds, or None if no solution exists
    """

def find_earliest_firing_solution(environment: Environment, projectile: Projectile, platform: Platform, target_position: Callable[[float], Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], time_range: tuple[float, float], integration_options: IntegrationOptions = ..., targeting_options: TargetingOptions = ...) -> TargetingSolution | None:
    """
    Find earliest firing solution for intercepting a target.

    Combines find_earliest_time with find_best_angles to produce a complete
    targeting solution at the earliest possible intercept time.

    Args:
        environment: Environmental conditions
        projectile: Projectile properties
        platform: Launch platform
        target_position: Function returning target position at given time
        time_range: Tuple of (min_time, max_time) in seconds
        integration_options: Integration parameters
        targeting_options: Targeting configuration

    Returns:
        Complete firing solution, or None if no solution exists
    """

def find_latest_firing_solution(environment: Environment, projectile: Projectile, platform: Platform, target_position: Callable[[float], Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]], time_range: tuple[float, float], integration_options: IntegrationOptions = ..., targeting_options: TargetingOptions = ...) -> TargetingSolution | None:
    """
    Find latest firing solution for intercepting a target.

    Combines find_latest_time with find_best_angles to produce a complete
    targeting solution at the latest possible intercept time.

    Args:
        environment: Environmental conditions
        projectile: Projectile properties
        platform: Launch platform
        target_position: Function returning target position at given time
        time_range: Tuple of (min_time, max_time) in seconds
        integration_options: Integration parameters
        targeting_options: Targeting configuration

    Returns:
        Complete firing solution, or None if no solution exists
    """

def bracket_find_root(f: Callable[[float], float], a_x: float, b_x: float, max_iter: int = 16) -> tuple[float, float]:
    """
    Find a bracketing interval containing a root using TOMS748 algorithm.

    Returns an interval [a, b] where f(a) and f(b) have opposite signs,
    guaranteeing a root exists within the interval by the intermediate value theorem.

    Args:
        f: Objective function to find root of
        a_x: Lower bound of search interval
        b_x: Upper bound of search interval
        max_iter: Maximum number of iterations (default: 32)

    Returns:
        Bracket (a, b) containing the root

    Note:
        f(a_x) and f(b_x) must have opposite signs
    """

def basin_find_minima(f: Callable[[float], float], lo_x: float, hi_x: float, max_iter: int = 16) -> tuple[float, float]:
    """
    Find minimum of a unimodal (single basin) function using Brent's method.

    Efficiently finds the global minimum for convex or unimodal functions.
    Uses parabolic interpolation with golden section search as fallback.

    Args:
        f: Objective function to minimize
        lo_x: Lower bound of search interval
        hi_x: Upper bound of search interval
        max_iter: Maximum number of iterations (default: 32)

    Returns:
        Tuple of (x_min, f(x_min)) - location and value of minimum
    """

def sinlike_find_minima(f: Callable[[float], float], lo_x: float, hi_x: float, h: float = 0.001, max_iter: int = 16) -> tuple[float, float]:
    """
    Find global minimum of a sine-like periodic function with one complete wave.

    Specialized algorithm for functions with approximately sinusoidal behavior
    containing a single global minimum per period. Uses boundary sampling to
    detect topology, then partitions the domain via root-finding and applies
    Brent's method to each subregion.

    Algorithm strategy:
    1. Sample near boundaries to detect if minimum is trivial (at boundary)
    2. If function is monotonic, search entire interval
    3. If multimodal, find level curve crossing and partition domain
    4. Apply basin optimization to each partition independently

    Args:
        f: Objective function to minimize (should be sine-like)
        lo_x: Lower bound of search interval
        hi_x: Upper bound of search interval
        h: Step size for boundary probing (default: 1e-3)
        max_iter: Maximum iterations per optimization call (default: 32)

    Returns:
        Tuple of (x_min, f(x_min)) - global minimum

    Note:
        - Assumes function has at most one complete oscillation in [lo_x, hi_x]
        - Requires hi_x - lo_x > 2*h for proper boundary sampling
    """
