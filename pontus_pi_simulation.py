"""Continuous-time PI study of a reduced Pontus surge model.

This program is intentionally independent of ROS, Gazebo, and Simulink.  It
uses values published in Pontus' vehicle model:
https://github.com/gt-marine-robotics-group/Pontus/blob/6b9513851edb717567bb1a9e555f27a521ec036d/pontus_description/models/pontus/pontus.xacro

It compares three PI controllers around a selected 0.4 m/s simulator trim.
The base-link mass parameter is used as the reduced effective mass. Additional
link inertias, coupling, thruster deadzone and limits are omitted. The existing
controller's variable feedforward is replaced here by a fixed trim-force bias.
This is not a reproduction of the complete Gazebo or ROS control system.

Run on Windows from this folder with:
    py pontus_pi_simulation.py
"""

from pathlib import Path
import math
import os
import json

# Save plots without requiring a desktop GUI or the optional Tcl/Tk runtime.
os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".matplotlib"))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# Published Pontus simulator geometry and hydrodynamic entries.
WATER_DENSITY = 1025.0  # kg/m^3
BODY_LENGTH = 0.61  # m
BODY_RADIUS = 0.10  # m
BATTERY_TUBE_LENGTH = 0.324  # m
BATTERY_TUBE_RADIUS = 0.045  # m
SIM_MASS_FACTOR = 0.95
LINEAR_SURGE_DRAG = 5.0  # magnitude of xU, N s/m
QUADRATIC_SURGE_DRAG = 6.2282  # magnitude of xUabsU, N s^2/m^2

# Selected local trim condition for this numerical example.
V_TRIM = 0.40  # m/s
DT = 0.01  # s
T_END = 75.0  # s

# Exactly three reference changes, all near the selected trim speed.
STEP_SCHEDULE = ((0.0, 0.40), (10.0, 0.30), (30.0, 0.50), (50.0, 0.40))

# Pole-cancelling candidates: lambda, Kp=lambda, Ki=(D_local/M)*lambda.
CANDIDATE_POLES = (0.40, 0.50, 0.67)
SELECTED_RATE = 0.67  # Fastest of these three candidates; not a global optimum.


def simulated_mass() -> float:
    """Base-link mass expression for sim=true, adopted as reduced-model mass."""
    body_volume = math.pi * BODY_LENGTH * BODY_RADIUS**2
    tube_volume = math.pi * BATTERY_TUBE_LENGTH * BATTERY_TUBE_RADIUS**2
    displaced_volume = body_volume + 2.0 * tube_volume
    return SIM_MASS_FACTOR * WATER_DENSITY * displaced_volume


MASS = simulated_mass()


def drag_force(speed: float) -> float:
    """Forward drag magnitude with its sign, opposing the direction of motion."""
    return LINEAR_SURGE_DRAG * speed + QUADRATIC_SURGE_DRAG * abs(speed) * speed


F_TRIM = drag_force(V_TRIM)
D_LOCAL = LINEAR_SURGE_DRAG + 2.0 * QUADRATIC_SURGE_DRAG * abs(V_TRIM)
PLANT_POLE = D_LOCAL / MASS


def reference_speed(time: float) -> float:
    """Return the piecewise-constant requested forward speed."""
    value = STEP_SCHEDULE[0][1]
    for step_time, step_value in STEP_SCHEDULE:
        if time >= step_time:
            value = step_value
        else:
            break
    return value


def state_derivative(state: np.ndarray, reference: float, kp: float, ki: float,
                     nonlinear: bool = True) -> np.ndarray:
    """Derivatives of absolute speed V and integrated tracking error I.

    The reference stays constant throughout each RK4 integration interval.
    This prevents a new step affecting the state before its scheduled time.
    """
    speed, integral_error = state
    error = reference - speed
    acceleration_command = kp * error + ki * integral_error
    equivalent_forward_force = F_TRIM + MASS * acceleration_command
    drag = drag_force(speed) if nonlinear else F_TRIM + D_LOCAL * (speed - V_TRIM)
    acceleration = (equivalent_forward_force - drag) / MASS
    return np.array((acceleration, error))


def run_case(closed_loop_pole: float, dt: float = DT,
             nonlinear: bool = True) -> dict[str, np.ndarray | float]:
    """Integrate a positive decay rate lambda (the pole is at -lambda).

    Each scheduled change must coincide with a time-grid point. This keeps
    integration intervals entirely inside one constant-reference segment.
    The time step is numerical accuracy, not a designed digital controller.
    """
    if closed_loop_pole <= 0 or dt <= 0:
        raise ValueError("The decay rate and integration step must be positive.")
    boundaries = [t for t, _ in STEP_SCHEDULE] + [T_END]
    if not all(math.isclose(t / dt, round(t / dt), abs_tol=1e-9, rel_tol=0)
               for t in boundaries):
        raise ValueError("Reference changes and T_END must align with the time grid.")
    kp = closed_loop_pole
    ki = PLANT_POLE * kp
    time = np.arange(round(T_END / dt) + 1) * dt
    reference = np.full(time.size, STEP_SCHEDULE[0][1], dtype=float)
    for step_time, step_value in STEP_SCHEDULE:
        reference[round(step_time / dt):] = step_value
    state = np.zeros((time.size, 2))
    state[0] = (V_TRIM, 0.0)

    for index in range(time.size - 1):
        y = state[index]
        r = reference[index]
        k1 = state_derivative(y, r, kp, ki, nonlinear)
        k2 = state_derivative(y + dt * k1 / 2.0, r, kp, ki, nonlinear)
        k3 = state_derivative(y + dt * k2 / 2.0, r, kp, ki, nonlinear)
        k4 = state_derivative(y + dt * k3, r, kp, ki, nonlinear)
        state[index + 1] = y + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0

    speed = state[:, 0]
    integral_error = state[:, 1]
    error = reference - speed
    acceleration_command = kp * error + ki * integral_error
    force = F_TRIM + MASS * acceleration_command

    return {
        "lambda": closed_loop_pole,
        "kp": kp,
        "ki": ki,
        "time": time,
        "reference": reference,
        "speed": speed,
        "acceleration_command": acceleration_command,
        "force": force,
        "integral_error": integral_error,
    }


def step_metrics(case: dict[str, np.ndarray | float]) -> list[dict[str, float]]:
    """Measure settling time and overshoot for each of the three changes."""
    time = case["time"]
    speed = case["speed"]
    metrics = []
    for index in range(1, len(STEP_SCHEDULE)):
        start_time, target = STEP_SCHEDULE[index]
        previous = STEP_SCHEDULE[index - 1][1]
        end_time = STEP_SCHEDULE[index + 1][0] if index + 1 < len(STEP_SCHEDULE) else T_END
        delta = target - previous
        mask = (time >= start_time) & (time <= end_time)
        t_segment = time[mask]
        v_segment = speed[mask]
        tolerance = 0.02 * abs(delta)
        within_band = np.abs(v_segment - target) <= tolerance
        settled_time = float("nan")
        for sample in range(within_band.size):
            if np.all(within_band[sample:]):
                settled_time = t_segment[sample] - start_time
                break
        overshoot = max(0.0, np.max((v_segment - target) * np.sign(delta)))
        metrics.append({"settling_time": float(settled_time), "overshoot": float(overshoot)})
    return metrics


def print_summary(cases: list[dict[str, np.ndarray | float]]) -> None:
    print("Reduced Pontus surge model: continuous-time PI comparison")
    print(f"Reduced-model mass (base-link parameter): {MASS:.4f} kg")
    print(f"Trim speed and force: {V_TRIM:.2f} m/s, {F_TRIM:.4f} N")
    print(f"Local drag slope: {D_LOCAL:.4f} N s/m; plant pole: -{PLANT_POLE:.5f} /s")
    print("\nCandidate results (equivalent net surge force):")
    for case in cases:
        metrics = step_metrics(case)
        settling = [m["settling_time"] for m in metrics]
        overshoot = [m["overshoot"] for m in metrics]
        print(
            f"  lambda={case['lambda']:.2f}: Kp={case['kp']:.4f}, Ki={case['ki']:.4f}, "
            f"peak |F_T|={np.max(np.abs(case['force'])):.3f} N, "
            f"settling={np.array2string(np.array(settling), precision=2)}, "
            f"overshoot={np.array2string(np.array(overshoot), precision=4)}"
        )
    print(
        "\nIdeal equivalent force input: no thruster allocation, deadzone or saturation. "
        "The PI output is an increment; total force = trim force + mass * PI output."
    )


def plot_comparison(cases: list[dict[str, np.ndarray | float]], output_dir: Path) -> None:
    """Save a candidate comparison plot and a selected-candidate plot."""
    colors = ("#404040", "#7A4E2D", "#2F6B55")
    plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 6.4), constrained_layout=True)
    for axis in axes:
        axis.grid(True, alpha=0.25)

    reference = cases[0]["reference"]
    time = cases[0]["time"]
    axes[0].step(time, reference, where="post", color="black", linewidth=2.0, label="Reference")
    for case, color in zip(cases, colors):
        axes[0].plot(time, case["speed"], color=color, linewidth=1.5, label=f"λ={case['lambda']:.2f} s⁻¹")
        axes[1].plot(time, case["acceleration_command"], color=color, linewidth=1.5, label=f"lambda={case['lambda']:.2f}")
        axes[2].plot(time, case["force"], color=color, linewidth=1.5, label=f"lambda={case['lambda']:.2f}")

    axes[0].set_ylabel("Forward speed (m/s)")
    axes[1].set_ylabel("Incremental PI\ncommand (m/s²)")
    axes[2].set_ylabel("Equivalent net\nthrust (N)")
    axes[2].set_xlabel("Time (s)")
    axes[0].set_title("Reduced Pontus surge model: PI candidate comparison")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside upper center", ncol=4, fontsize=10, frameon=False)
    for axis in axes:
        axis.set_xlim(0, T_END)
    fig.savefig(output_dir / "pontus_pi_candidate_comparison.png", dpi=180)
    plt.close(fig)

    # The fastest candidate is shown separately for the client-facing three-step plot.
    selected = next(case for case in cases if math.isclose(case["lambda"], SELECTED_RATE))
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 7.2), constrained_layout=True)
    for axis in axes:
        axis.grid(True, alpha=0.25)
    axes[0].step(selected["time"], selected["reference"], where="post", color="black", linewidth=2.0, label="Requested speed")
    axes[0].plot(selected["time"], selected["speed"], color="#2F6B55", linewidth=1.8, label="Plant response")
    axes[1].plot(selected["time"], selected["acceleration_command"], color="#7A4E2D", linewidth=1.5)
    axes[2].plot(selected["time"], selected["force"], color="#404040", linewidth=1.5)
    axes[0].set_ylabel("Forward speed (m/s)")
    axes[1].set_ylabel("Incremental PI\ncommand (m/s²)")
    axes[2].set_ylabel("Equivalent net\nthrust (N)")
    axes[2].set_xlabel("Time (s)")
    axes[0].legend(loc="best")
    axes[0].set_title(
        f"Reduced Pontus surge model: three reference changes\n"
        f"Kp={selected['kp']:.3f} s⁻¹, Ki={selected['ki']:.4f} s⁻², Kd=0"
    )
    for axis in axes:
        axis.set_xlim(0, T_END)
        for step_time, _ in STEP_SCHEDULE[1:]:
            axis.axvline(step_time, color="0.65", linestyle=":", linewidth=0.8)
    axes[2].axhline(F_TRIM, color="0.4", linestyle="--", linewidth=0.8, label="Trim force")
    axes[2].legend(loc="best", fontsize=9)
    fig.savefig(output_dir / "pontus_pi_selected_response.png", dpi=180)
    plt.close(fig)

    csv = np.column_stack(
        (
            selected["time"],
            selected["reference"],
            selected["speed"],
            selected["acceleration_command"],
            selected["force"],
            selected["integral_error"],
        )
    )
    np.savetxt(
        output_dir / "pontus_pi_selected_response.csv",
        csv,
        delimiter=",",
        header="time_s,reference_speed_mps,plant_speed_mps,incremental_pi_command_mps2,equivalent_net_thrust_N,integral_error_m",
        comments="",
    )


def main() -> None:
    output_dir = Path(__file__).resolve().parent
    cases = [run_case(pole) for pole in CANDIDATE_POLES]
    print_summary(cases)
    plot_comparison(cases, output_dir)
    summary = {
        "mass_kg": MASS, "trim_speed_mps": V_TRIM, "trim_force_N": F_TRIM,
        "local_drag_Ns_per_m": D_LOCAL, "plant_decay_rate_per_s": PLANT_POLE,
        "dt_s": DT, "selected_rate_per_s": SELECTED_RATE,
        "candidates": [{"lambda": c["lambda"], "kp": c["kp"], "ki": c["ki"],
                        "peak_force_N": float(np.max(np.abs(c["force"]))),
                        "steps": step_metrics(c)} for c in cases],
    }
    (output_dir / "simulation_results.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"\nSaved plots and CSV in: {output_dir}")


if __name__ == "__main__":
    main()
