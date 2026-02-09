"""Test utilities for ballistic solve testing."""
import numpy as np
from math import factorial


def polynomial_trajectory(coeffs):
    """Create a polynomial trajectory function from coefficients.
    
    Args:
        coeffs: List of coefficient vectors for each polynomial term
        
    Returns:
        Function that computes trajectory position at time t
    """
    return lambda t: sum(c * (t**n) / factorial(n) for n, c in enumerate(coeffs))


def random_unit_vector():
    """Generate a random unit vector in 3D space.
    
    Returns:
        Normalized 3D vector
    """
    v = np.random.normal(0, 1, 3)
    return v / np.linalg.norm(v)


def generate_test_case(ballistic):
    """Generate a random test case for ballistic solving.
    
    Args:
        ballistic: Ballistic solver instance
        
    Returns:
        Tuple of (platform_position, platform_velocity, projectile_speed, 
                  target_position_func, expected_time)
    """
    platform_position = np.random.uniform(-5000, 5000, 3)
    platform_velocity = np.random.uniform(-2000, 2000, 3)
    projectile_speed = np.random.uniform(500, 2000)
    direction = random_unit_vector()
    time = np.random.uniform(0.0, 15.0)

    scales = [0, 500, 100, 20, 5, 1]
    coeffs = [np.random.uniform(-scale, scale, 3) for scale in scales]
    trajectory = polynomial_trajectory(coeffs)

    intercept = ballistic.simulate(
        platform_position, platform_velocity, projectile_speed, direction, time
    )

    return (
        platform_position,
        platform_velocity,
        projectile_speed,
        lambda t: trajectory(t) - trajectory(time) + intercept,
        time,
    )
