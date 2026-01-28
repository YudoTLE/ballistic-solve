"""Ballistic trajectory simulation and optimization library"""

from ._core import (
    # Core classes
    Environment,
    Projectile,
    Platform,
    Trajectory,
    IntegrationOptions,
    TargetingSolution,
    TargetingOptions,
    AngleSearchStrategy,
    
    # Utility functions
    to_direction,
    to_angles,
    
    # Trajectory computation
    compute_trajectory,
    compute_point,
    
    # Targeting functions
    find_best_azimuth_angle,
    find_best_elevation_angle,
    find_best_angles,
    find_best_direction,
    find_earliest_time,
    find_latest_time,
    find_earliest_firing_solution,
    find_latest_firing_solution,
    
    # Optimization tools
    bracket_find_root,
    basin_find_minima,
    sinlike_find_minima,
)

__version__ = "0.0.0"

__all__ = [
    "Environment",
    "Projectile",
    "Platform",
    "Trajectory",
    "IntegrationOptions",
    "TargetingSolution",
    "TargetingOptions",
    "AngleSearchStrategy",
    "to_direction",
    "to_angles",
    "compute_trajectory",
    "compute_point",
    "find_best_azimuth_angle",
    "find_best_elevation_angle",
    "find_best_angles",
    "find_best_direction",
    "find_earliest_time",
    "find_latest_time",
    "find_earliest_firing_solution",
    "find_latest_firing_solution",
    "bracket_find_root",
    "basin_find_minima",
    "sinlike_find_minima",
]