<div align="center">

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/images/hero-dark.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/images/hero-dark.png">
  <img alt="ballistic-solve" src="docs/images/hero-light.png" width="70%">
</picture>

[![Python 3.12+](https://img.shields.io/badge/python-3.12+-blue.svg)](https://www.python.org/downloads/)
[![C++](https://img.shields.io/badge/C++-20-00599C.svg)](https://isocpp.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

*Real-time numerical solver for moving target interception with realistic ballistic trajectory*

</div>

## Overview

**ballistic-solve** is a real-time C++ library with Python bindings for solving realistic ballistic trajectory problems. It computes optimal firing solutions to intercept stationary or moving targets while accounting for real-world physics including gravity, aerodynamic drag, wind effects, and atmospheric variations.

### Key Features

- **Real-time Performance** — High performance computation with very low latency
- **Moving Target Interception** — Solve for earliest/latest intercept solutions against **any continous** moving targets
- **Environmental Modeling** — ISA atmosphere, variable air density, 3D wind fields, temperature gradients
- **Velocity-Dependent Drag** — Support for custom drag coefficient functions (Mach-dependent Cd)
- **Python Bindings** — Full NumPy integration via nanobind for seamless scientific workflows

---

## Installation

```bash
pip install ballistic-solve
```

---

## Quick Start

```python
import numpy as np
from ballistic_solve import Ballistic, Environment, Projectile

# Configure environment (defaults to ISA atmosphere with standard gravity)
env = Environment()

# Define projectile: 10g bullet, 7.62mm diameter, Cd=0.295
projectile = Projectile(
    mass=0.010,
    area=np.pi * (0.00762 / 2) ** 2,
    drag_coefficient=0.295
)

# Create solver
solver = Ballistic(env, projectile)

# Simulate trajectory
computed_point = solver.simulate(
    platform_position=np.array([0.0, 0.0, 0.0]),
    platform_velocity=np.array([0.0, 0.0, 0.0]),
    projectile_speed=850.0,
    angles=np.array([0, np.pi * 0.5]),
    time=3.0
)

print(f"Projectile at {computed_point} after 3s")
```

### Solving for Moving Target Intercept

```python
# Example target trajectory
def target_position(t):
    initial_pos = np.array([500.0, 200.0, 50.0])
    velocity = np.array([15.0, -5.0, 0.0])
    acceleration = np.array([2.0, -1.0, 0.5])

    return initial_pos + velocity * t + 0.5 * acceleration * t**2

# Find earliest intercept solution
solution = solver.solve_earliest(
    target_position=target_position,
    platform_position=np.array([0.0, 0.0, 0.0]),
    platform_velocity=np.array([0.0, 0.0, 0.0]),
    projectile_speed=850.0,
    time_range=(0.0, 5.0)
)

if solution:
    print(f"Fire direction: {solution.direction}")
    print(f"Time to intercept: {solution.time:.3f}s")
```

---

## Core Concepts

### Environment

Models atmospheric and gravitational conditions:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `gravity` | Gravity vector (m/s²) | `[0, 0, -9.81]` |
| `air_density` | Position-dependent density (kg/m³) | ISA model |
| `wind_velocity` | 3D wind field (m/s) | Zero wind |
| `temperature` | Position-dependent temperature (K) | ISA model |

### Projectile

Defines projectile aerodynamics:

| Parameter | Description |
|-----------|-------------|
| `mass` | Projectile mass (kg) |
| `area` | Cross-sectional area (m²) |
| `drag_coefficient` | Cd value or Cd(velocity) function |

### Solver Methods

| Method | Description |
|--------|-------------|
| `simulate()` | Compute trajectory over time range or at specific time |
| `solve_earliest()` | Find first possible intercept solution |
| `solve_latest()` | Find last possible intercept solution |
| `find_best_angles()` | Optimize azimuth/elevation given simulation time  |

---

## Dependencies

- **Eigen** — Vectorized linear algebra
- **Boost** — Adaptive-step ODE integration and root-finding

---

## License

[MIT](LICENSE) © Ariyudo Pertama