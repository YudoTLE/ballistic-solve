import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import random
import time

import _core as bs

# ============================================================
# ENVIRONMENT SETUP
# ============================================================

# Create environment with custom air density function
def air_density_func(h: float) -> float:
    """Air density as function of altitude (single parameter)"""
    return 1.225 * np.exp(-h / 8500)

# Create environment - using builder pattern
environment = (
    bs.Environment()
)

print(environment.gravity)

# Alternative: Create environment with spatial functions
# def air_density_spatial(pos: np.ndarray) -> float:
#     """Air density as function of 3D position"""
#     return 1.225 * np.exp(-pos[2] / 8500)
# 
# def wind_velocity_spatial(pos: np.ndarray) -> np.ndarray:
#     """Wind velocity as function of 3D position"""
#     return np.array([5.0 + pos[2] * 0.01, 2.0, 0.0])
# 
# environment = bs.Environment(
#     gravity=np.array([0.0, 0.0, -9.81]),
#     air_density=air_density_spatial,
#     wind_velocity=wind_velocity_spatial,
#     temperature=lambda pos: 288.15 - pos[2] * 0.0065  # Temperature lapse
# )

# Create projectile
projectile = (
    bs.Projectile()
    .with_mass(100.0)
    .with_area(1.0)
    .with_drag_coefficient(0.47)
)

# Create ballistic solver with integration and targeting options
integration_options = bs.Integration(
    max_step_size=1000.0,
    initial_step_size=0.001,
    absolute_tolerance=1e-6,
    relative_tolerance=1e-6
)

targeting_options = bs.Targeting(
    h=1e-3,  # Finite difference step
    time_scan_interval=0.5,  # Time scan step
    angle_max_iteration=16,
    time_max_iteration=16
)

ballistic_solver = bs.Ballistic(
    environment,
    projectile,
    integration_options,
    targeting_options
)

# Platform parameters
platform_position = np.array([0.0, 0.0, 0.0])
platform_velocity = np.array([0.0, 0.0, 0.0])
muzzle_velocity = 2200.0

# ============================================================
# TARGET MODEL (UP TO JERK)
# p(t) = p0 + v t + 1/2 a t^2 + 1/6 j t^3
# ============================================================


def make_moving_target(p0, v0, a0, j0):
    """Create a moving target with position, velocity, acceleration, and jerk."""
    p0 = np.asarray(p0, float)
    v0 = np.asarray(v0, float)
    a0 = np.asarray(a0, float)
    j0 = np.asarray(j0, float)

    def target_position(t: float) -> np.ndarray:
        return p0 + v0 * t + 0.5 * a0 * t**2 + (1.0 / 6.0) * j0 * t**3

    return target_position


def random_target():
    """Generate a random challenging target with complex motion."""
    return make_moving_target(
        p0=[
            random.uniform(300, 1200),
            random.uniform(-1200, 1200),
            random.uniform(100, 800),
        ],
        v0=[
            random.uniform(-200, 200),
            random.uniform(-200, 200),
            random.uniform(-50, 50),
        ],
        a0=[
            random.uniform(-30, 30),
            random.uniform(-30, 30),
            random.uniform(-30, 0),
        ],
        j0=[
            random.uniform(-20, 20),
            random.uniform(-20, 20),
            random.uniform(-20, 20),
        ],
    )


# ============================================================
# RUN STRESS TEST
# ============================================================

N_TESTS = 100
TIME_RANGE = (0.0, 30.0)

# Statistics storage
errors = []
computation_times = []
intercept_times = []
solved_indices = []
failed_indices = []

# 3D trajectory plot
fig_3d = go.Figure()

print("Running ballistic intercept stress test...")
print("=" * 60)

for i in range(N_TESTS):
    target_position = random_target()

    # Time the computation
    start_time = time.perf_counter()

    # Use the new solve_latest method
    firing_solution = ballistic_solver.solve_latest(
        target_position=target_position,
        platform_position=platform_position,
        platform_velocity=platform_velocity,
        projectile_speed=muzzle_velocity,
        time_range=TIME_RANGE
    )

    end_time = time.perf_counter()
    comp_time_ms = (end_time - start_time) * 1000.0

    if firing_solution is None:
        print(f"[{i+1:03d}/{N_TESTS}] FAILED - No solution found")
        failed_indices.append(i)
        continue

    # Compute trajectory using the simulate method
    traj = ballistic_solver.simulate(
        platform_position=platform_position,
        platform_velocity=platform_velocity,
        projectile_speed=muzzle_velocity,
        direction=firing_solution.direction,
        time_range=(0, firing_solution.time)
    )

    positions = np.asarray(traj.positions)
    times = np.asarray(traj.times)

    impact = positions[-1]
    error = np.linalg.norm(impact - target_position(firing_solution.time))

    # Store statistics
    errors.append(error)
    computation_times.append(comp_time_ms)
    intercept_times.append(firing_solution.time)
    solved_indices.append(i)

    print(
        f"[{i+1:03d}/{N_TESTS}] ✓ t={firing_solution.time:.3f}s  "
        f"error={error:.2e}m  calc={comp_time_ms:.2f}ms"
    )

    # Plot projectile trajectory (colorful)
    fig_3d.add_trace(
        go.Scatter3d(
            x=positions[:, 0],
            y=positions[:, 1],
            z=positions[:, 2],
            mode="lines",
            line=dict(width=2),
            showlegend=False,
            hovertemplate="Projectile<br>x=%{x:.1f}<br>y=%{y:.1f}<br>z=%{z:.1f}<extra></extra>",
        )
    )

    # Plot target trajectory (colorful)
    target_traj = np.array([target_position(t) for t in times])
    fig_3d.add_trace(
        go.Scatter3d(
            x=target_traj[:, 0],
            y=target_traj[:, 1],
            z=target_traj[:, 2],
            mode="lines",
            line=dict(width=2, dash="dash"),
            showlegend=False,
            hovertemplate="Target<br>x=%{x:.1f}<br>y=%{y:.1f}<br>z=%{z:.1f}<extra></extra>",
        )
    )

    # Mark impact point
    fig_3d.add_trace(
        go.Scatter3d(
            x=[impact[0]],
            y=[impact[1]],
            z=[impact[2]],
            mode="markers",
            marker=dict(size=4, color="red"),
            showlegend=False,
            hovertemplate=f"Impact #{i+1}<br>error={error:.2e}m<extra></extra>",
        )
    )

# ============================================================
# COMPUTE STATISTICS
# ============================================================

errors = np.array(errors)
computation_times = np.array(computation_times)
intercept_times = np.array(intercept_times)

n_solved = len(errors)
n_failed = len(failed_indices)
success_rate = (n_solved / N_TESTS) * 100

print("\n" + "=" * 60)
print("STRESS TEST SUMMARY")
print("=" * 60)
print(f"Total Tests:     {N_TESTS}")
print(f"Solved:          {n_solved} ({success_rate:.1f}%)")
print(f"Failed:          {n_failed} ({100-success_rate:.1f}%)")
print()
print("ERROR STATISTICS (meters):")
print(f"  RMSE:          {np.sqrt(np.mean(errors**2)):.4e}")
print(f"  Mean:          {np.mean(errors):.4e}")
print(f"  Median:        {np.median(errors):.4e}")
print(f"  Min:           {np.min(errors):.4e}")
print(f"  Max:           {np.max(errors):.4e}")
print(f"  Std Dev:       {np.std(errors):.4e}")
print()
print("COMPUTATION TIME (ms):")
print(f"  Mean:          {np.mean(computation_times):.2f}")
print(f"  Median:        {np.median(computation_times):.2f}")
print(f"  Min:           {np.min(computation_times):.2f}")
print(f"  Max:           {np.max(computation_times):.2f}")
print(f"  Total:         {np.sum(computation_times):.2f}")
print()
print("INTERCEPT TIME (seconds):")
print(f"  Mean:          {np.mean(intercept_times):.3f}")
print(f"  Median:        {np.median(intercept_times):.3f}")
print(f"  Min:           {np.min(intercept_times):.3f}")
print(f"  Max:           {np.max(intercept_times):.3f}")
print("=" * 60)

# ============================================================
# CREATE COMPREHENSIVE PLOTS
# ============================================================

# 3D Trajectory Plot
fig_3d.update_layout(
    title=dict(
        text=f"Ballistic Intercept Stress Test - 3D Trajectories<br>"
        f"<sub>Success: {n_solved}/{N_TESTS} ({success_rate:.1f}%), "
        f"RMSE: {np.sqrt(np.mean(errors**2)):.2e}m</sub>",
        x=0.5,
        xanchor="center",
    ),
    scene=dict(
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        zaxis_title="Z (m)",
        aspectmode="data",
    ),
    height=700,
)

# Statistics Dashboard
fig_stats = make_subplots(
    rows=3,
    cols=2,
    subplot_titles=(
        "Error Distribution (Histogram)",
        "Error by Test Number",
        "Computation Time Distribution",
        "Computation Time by Test Number",
        "Intercept Time Distribution",
        "Error vs Intercept Time",
    ),
    specs=[
        [{"type": "histogram"}, {"type": "scatter"}],
        [{"type": "histogram"}, {"type": "scatter"}],
        [{"type": "histogram"}, {"type": "scatter"}],
    ],
    vertical_spacing=0.12,
    horizontal_spacing=0.12,
)

# Row 1: Error plots
fig_stats.add_trace(
    go.Histogram(
        x=errors,
        nbinsx=30,
        name="Error",
        marker_color="indianred",
        showlegend=False,
    ),
    row=1,
    col=1,
)

fig_stats.add_trace(
    go.Scatter(
        x=solved_indices,
        y=errors,
        mode="markers",
        marker=dict(
            size=6,
            color=errors,
            colorscale="Viridis",
            showscale=True,
            colorbar=dict(title="Error (m)", x=1.15),
        ),
        name="Error",
        showlegend=False,
    ),
    row=1,
    col=2,
)

# Add RMSE, min, max lines
rmse = np.sqrt(np.mean(errors**2))
fig_stats.add_hline(
    y=rmse,
    line_dash="dash",
    line_color="red",
    annotation_text=f"RMSE: {rmse:.2e}m",
    row=1,
    col=2,
)
fig_stats.add_hline(
    y=np.min(errors),
    line_dash="dot",
    line_color="green",
    annotation_text=f"Min: {np.min(errors):.2e}m",
    row=1,
    col=2,
)
fig_stats.add_hline(
    y=np.max(errors),
    line_dash="dot",
    line_color="orange",
    annotation_text=f"Max: {np.max(errors):.2e}m",
    row=1,
    col=2,
)

# Row 2: Computation time plots
fig_stats.add_trace(
    go.Histogram(
        x=computation_times,
        nbinsx=30,
        name="Comp Time",
        marker_color="steelblue",
        showlegend=False,
    ),
    row=2,
    col=1,
)

fig_stats.add_trace(
    go.Scatter(
        x=solved_indices,
        y=computation_times,
        mode="markers",
        marker=dict(size=6, color="steelblue"),
        name="Comp Time",
        showlegend=False,
    ),
    row=2,
    col=2,
)

fig_stats.add_hline(
    y=np.mean(computation_times),
    line_dash="dash",
    line_color="blue",
    annotation_text=f"Mean: {np.mean(computation_times):.2f}ms",
    row=2,
    col=2,
)

# Row 3: Intercept time and correlation
fig_stats.add_trace(
    go.Histogram(
        x=intercept_times,
        nbinsx=30,
        name="Intercept Time",
        marker_color="seagreen",
        showlegend=False,
    ),
    row=3,
    col=1,
)

fig_stats.add_trace(
    go.Scatter(
        x=intercept_times,
        y=errors,
        mode="markers",
        marker=dict(
            size=8,
            color=computation_times,
            colorscale="Plasma",
            showscale=True,
            colorbar=dict(title="Calc Time (ms)", x=1.15, y=0.15, len=0.3),
        ),
        name="Error vs Time",
        showlegend=False,
    ),
    row=3,
    col=2,
)

# Update axes labels
fig_stats.update_xaxes(title_text="Error (m)", row=1, col=1)
fig_stats.update_xaxes(title_text="Test Number", row=1, col=2)
fig_stats.update_xaxes(title_text="Computation Time (ms)", row=2, col=1)
fig_stats.update_xaxes(title_text="Test Number", row=2, col=2)
fig_stats.update_xaxes(title_text="Intercept Time (s)", row=3, col=1)
fig_stats.update_xaxes(title_text="Intercept Time (s)", row=3, col=2)

fig_stats.update_yaxes(title_text="Count", row=1, col=1)
fig_stats.update_yaxes(title_text="Error (m)", row=1, col=2)
fig_stats.update_yaxes(title_text="Count", row=2, col=1)
fig_stats.update_yaxes(title_text="Computation Time (ms)", row=2, col=2)
fig_stats.update_yaxes(title_text="Count", row=3, col=1)
fig_stats.update_yaxes(title_text="Error (m)", row=3, col=2)

fig_stats.update_layout(
    title=dict(
        text=f"Ballistic Intercept Performance Analysis<br>"
        f"<sub>{N_TESTS} Tests | Success Rate: {success_rate:.1f}% | "
        f"Mean Calc Time: {np.mean(computation_times):.2f}ms</sub>",
        x=0.5,
        xanchor="center",
    ),
    height=1200,
    showlegend=False,
)

# ============================================================
# SHOW PLOTS
# ============================================================

fig_3d.show()
fig_stats.show()

print("\nPlots generated successfully!")