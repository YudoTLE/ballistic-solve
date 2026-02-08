import pytest
import numpy as np
from math import factorial
from ballistic_solve import Ballistic, Environment, Projectile


def polynomial_trajectory(coeffs):
    return lambda t: sum(c * (t**n) / factorial(n) for n, c in enumerate(coeffs))


def random_unit_vector():
    v = np.random.normal(0, 1, 3)
    return v / np.linalg.norm(v)


def generate_test_case(ballistic):
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


@pytest.fixture
def ballistic():
    environment = Environment.earth_standard()
    projectile = Projectile.gsh30_round()
    return Ballistic(environment, projectile)


def test_ballistic_solve_success_rate(ballistic):
    num_tests = 1000
    num_successes = 0

    np.random.seed(42)

    for i in range(num_tests):
        (
            platform_position,
            platform_velocity,
            projectile_speed,
            target_position,
            expected_time,
        ) = generate_test_case(ballistic)

        solution = ballistic.solve_earliest(
            target_position=target_position,
            platform_position=platform_position,
            platform_velocity=platform_velocity,
            projectile_speed=projectile_speed,
            time_range=(0.0, 20.0),
        )

        if solution:
            num_successes += 1

    success_rate = num_successes / num_tests

    assert (
        success_rate >= 0.91
    ), f"Success rate {success_rate:.2%} is below the required threshold"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
