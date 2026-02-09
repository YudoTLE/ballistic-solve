import pytest
import numpy as np
from ballistic_solve import Ballistic, Environment, Projectile
from test_utils import generate_test_case


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
