"""Read-only synchronized force and outer-loop diagnostics for N110D."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .force_regulation_outer import ForceRegulationConfig, diagnostics_arrays, replay_outer_loop


def analyze_trace(trace_path: Path, config: ForceRegulationConfig, *, window: int = 100) -> tuple[dict, dict[str, np.ndarray]]:
    with np.load(trace_path, allow_pickle=False) as archive:
        trace = {key: archive[key] for key in archive.files}
    force = np.asarray(trace["measured_normal_force_n"], dtype=float)
    desired = np.asarray(trace["desired_normal_force_n"], dtype=float)
    detected = np.asarray(trace["contact_detected"], dtype=bool)
    geometry = np.asarray(trace["contact_count"], dtype=int) > 0
    if len(force) != 500 or window != 100 or len(trace["task_time"]) != 50:
        raise ValueError("N110D requires the frozen 500-step / 50-tick / 100-sample window")
    outer = replay_outer_loop(trace, config)
    arrays = diagnostics_arrays(outer)
    x_error = float(np.max(np.abs(arrays["x_after_step_m"] - trace["command_normal_offset_m"])))
    v_error = float(np.max(np.abs(arrays["v_after_step_m_s"] - trace["command_normal_offset_velocity_m_s"])))
    time_error = float(np.max(np.abs(arrays["feedback_time_s"] - trace["feedback_time_s"])))
    force_error = float(np.max(np.abs(arrays["measured_force_n"] - trace["feedback_measured_normal_force_n"])))
    if max(x_error, v_error) > 1e-12 or max(time_error, force_error) > 1e-11:
        raise ValueError(f"outer reconstruction differs from trace: x={x_error}, v={v_error}, time={time_error}, force={force_error}")
    error = desired - force
    e = error[-window:]
    f = force[-window:]
    g = geometry[-window:]
    d = detected[-window:]
    zero = f == 0.0
    squared = e * e
    mse = float(np.mean(squared))
    mean_error = float(np.mean(e))
    variance = float(np.var(e))
    normal_tracking = np.einsum("ij,ij->i", trace["desired_position"] - trace["flange_position"],
                                 trace["contact_normal_world"])
    loss_rows = []
    for index in np.flatnonzero(arrays["loss_event"]):
        later = np.flatnonzero(arrays["detected_after"][index+1:])
        next_acquired = int(index+1+later[0]) if len(later) else None
        loss_rows.append({
            "index": int(index), "feedback_time_s": float(trace["feedback_time_s"][index]),
            "force_n": float(arrays["measured_force_n"][index]),
            "x_before_reset_m": float(arrays["x_before_reset_m"][index]),
            "v_before_reset_m_s": float(arrays["v_before_reset_m_s"][index]),
            "reset_applied": bool(arrays["reset_applied"][index]),
            "x_after_reset_m": float(arrays["x_after_reset_m"][index]),
            "v_after_reset_m_s": float(arrays["v_after_reset_m_s"][index]),
            "x_after_step_m": float(arrays["x_after_step_m"][index]),
            "v_after_step_m_s": float(arrays["v_after_step_m_s"][index]),
            "command_offset_step_change_m": float(arrays["x_after_step_m"][index] - arrays["x_after_step_m"][index-1]),
            "next_detected_index": next_acquired,
            "steps_until_next_detection": int(next_acquired-index) if next_acquired is not None else None,
        })
    contribution = lambda mask: float(np.sum(squared[mask]) / window)
    result = {
        "schema_version": "n110d_force_error_decomposition_v1",
        "trace_path": str(trace_path),
        "time_rule": "post-step force and desired at t_i; causal outer input at prior feedback t_{i-1}",
        "window": {"last_samples": window, "duration_s": window * config.timestep_s,
                   "first_time_s": float(trace["time"][-window]), "last_time_s": float(trace["time"][-1])},
        "full_window": {
            "desired_force_mean_n": float(np.mean(desired[-window:])),
            "measured_force_mean_n": float(np.mean(f)),
            "signed_error_mean_n": mean_error,
            "force_error_variance_n2": variance,
            "force_error_mse_n2": mse,
            "force_rmse_n": float(np.sqrt(mse)),
            "bias_variance_identity_error_n2": abs(mse - (mean_error**2 + variance)),
            "geometric_contact_fraction": float(np.mean(g)),
            "force_detected_fraction": float(np.mean(d)),
            "true_zero_force_fraction": float(np.mean(zero)),
            "zero_force_rmse_lower_bound_n": float(np.sqrt(np.sum(desired[-window:][zero]**2) / window)),
            "mse_contribution_geometric_contact_n2": contribution(g),
            "mse_contribution_geometric_loss_n2": contribution(~g),
            "mse_contribution_force_detected_n2": contribution(d),
            "mse_contribution_below_detection_n2": contribution(~d),
            "mse_contribution_true_zero_force_n2": contribution(zero),
            "mse_contribution_nonzero_force_n2": contribution(~zero),
            "geometric_loss_samples": int(np.sum(~g)),
            "below_detection_samples": int(np.sum(~d)),
            "true_zero_force_samples": int(np.sum(zero)),
        },
        "full_stage": {
            "desired_force_impulse_ns": float(np.sum(desired) * config.timestep_s),
            "actual_force_impulse_ns": float(np.sum(force) * config.timestep_s),
            "force_impulse_deficit_ns": float(np.sum(desired-force) * config.timestep_s),
            "geometric_contact_fraction": float(np.mean(geometry)),
            "force_detected_fraction": float(np.mean(detected)),
            "true_zero_force_fraction": float(np.mean(force == 0.0)),
            "ever_force_detected": bool(np.any(detected)),
            "outer_loss_events": int(np.sum(arrays["loss_event"])),
            "reset_applied_count": int(np.sum(arrays["reset_applied"])),
            "offset_limit_occupancy_fraction": float(np.mean(arrays["offset_clipped"])),
            "velocity_limit_occupancy_fraction": float(np.mean(arrays["velocity_clipped"])),
            "boundary_velocity_zero_count": int(np.sum(arrays["boundary_velocity_zeroed"])),
            "max_abs_offset_m": float(np.max(np.abs(arrays["x_after_step_m"]))),
            "max_abs_velocity_m_s": float(np.max(np.abs(arrays["v_after_step_m_s"]))),
        },
        "outer_reconstruction": {
            "maximum_offset_error_m": x_error, "maximum_velocity_error_m_s": v_error,
            "maximum_time_error_s": time_error, "maximum_feedback_force_error_n": force_error,
            "maximum_discrete_balance_error_n": float(np.max(np.abs(
                arrays["mass_times_acceleration_n"] + arrays["damping_force_n"]
                + arrays["spring_force_n"] - arrays["force_error_n"]))),
            "virtual_stiffness_n_m": config.stiffness_n_m,
            "explicit_damping_n_s_m": config.damping_n_s_m,
        },
        "inner_normal_execution": {
            "commanded_flange_normal_error_mean_m": float(np.mean(normal_tracking)),
            "commanded_flange_normal_error_rms_m": float(np.sqrt(np.mean(normal_tracking**2))),
            "commanded_flange_normal_error_max_abs_m": float(np.max(np.abs(normal_tracking))),
            "maximum_penetration_m": float(np.max(trace["penetration_m"])),
            "hqp_primary_linear_residual_rms_m_s": float(np.sqrt(np.mean(
                np.asarray(trace["task_primary_physical_linear_residual_m_s"], dtype=float)**2))),
            "hqp_task_success_fraction": float(np.mean(trace["task_success"])),
            "command_offset_is_not_physical_penetration": True,
        },
        "reset_events": loss_rows,
    }
    return result, arrays


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stiffness", type=float, default=2500.0)
    parser.add_argument("--reset", action="store_true")
    args = parser.parse_args()
    config = ForceRegulationConfig(1.0, 100.0, args.stiffness, 0.002, 0.003, 0.020, 0.20, 0.10, args.reset)
    result, arrays = analyze_trace(args.trace, config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    if args.output.name == "baseline_error_decomposition.json":
        np.savez_compressed(args.output.parent/"baseline_outer_diagnostics.npz", **arrays)
    print(json.dumps({"full_window": result["full_window"], "full_stage": result["full_stage"]}, indent=2))


if __name__ == "__main__":
    main()
