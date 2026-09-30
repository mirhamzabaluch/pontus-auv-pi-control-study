"""Independent numerical checks. Run: py verify_simulation.py"""
from pathlib import Path
import json
import numpy as np
import pontus_pi_simulation as model


def main():
    rate = model.SELECTED_RATE
    linear = model.run_case(rate, nonlinear=False)
    t = linear["time"]
    exact = np.full(t.size, model.V_TRIM)
    for i in range(1, len(model.STEP_SCHEDULE)):
        when, target = model.STEP_SCHEDULE[i]
        change = target - model.STEP_SCHEDULE[i - 1][1]
        active = t >= when
        exact[active] += change * (1 - np.exp(-rate * (t[active] - when)))
    analytic_error = float(np.max(np.abs(linear["speed"] - exact)))
    assert analytic_error < 1e-8, "Linear model disagrees with exact pole-cancelled response"

    coarse = model.run_case(rate)
    fine = model.run_case(rate, dt=model.DT / 2)
    grid_error = float(np.max(np.abs(coarse["speed"] - fine["speed"][::2])))
    assert grid_error < 1e-8, "Time-step convergence failed"
    first_step = round(model.STEP_SCHEDULE[1][0] / model.DT)
    assert np.all(coarse["speed"][:first_step + 1] == model.V_TRIM), "Premature response"
    assert np.all(coarse["integral_error"][:first_step + 1] == 0), "Premature integration"
    assert np.count_nonzero(np.diff(coarse["reference"])) == 3
    error = coarse["reference"] - coarse["speed"]
    expected_force = model.F_TRIM + model.MASS * (rate * error + coarse["ki"] * coarse["integral_error"])
    assert np.allclose(expected_force, coarse["force"], atol=1e-12, rtol=0)
    # Check the unreduced two-state model, retaining the cancelled stable pole.
    poles = np.linalg.eigvals([[-model.PLANT_POLE - rate, model.PLANT_POLE * rate], [-1, 0]])
    assert np.all(np.real(poles) < 0)
    assert np.all(np.abs(np.imag(poles)) < 1e-12)

    for candidate in model.CANDIDATE_POLES:
        case = model.run_case(candidate)
        assert np.all(np.isfinite(case["speed"]))
        assert all(np.isfinite(m["settling_time"]) for m in model.step_metrics(case))

    folder = Path(__file__).resolve().parent
    saved = np.loadtxt(folder / "pontus_pi_selected_response.csv", delimiter=",", skiprows=1)
    current = np.column_stack([coarse[k] for k in ("time", "reference", "speed", "acceleration_command", "force", "integral_error")])
    assert np.array_equal(saved, current), "Saved CSV is stale"
    report = {
        "status": "PASS",
        "exact_linear_solution_max_speed_error_mps": analytic_error,
        "halved_step_max_speed_difference_mps": grid_error,
        "full_linear_closed_loop_poles_per_s": sorted(float(p.real) for p in poles),
        "no_response_before_first_step": True,
        "saved_csv_matches_current_code": True,
        "three_reference_changes": True,
        "interpretation": "Numerical checks for the stated reduced model, not validation of the complete AUV.",
    }
    (folder / "verification_results.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
