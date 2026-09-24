"""Plot only the frozen, current-generation experiment results."""

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from plot_style import COLORS, ROOT, finish

BASE = ROOT / "output" / "fpmfc"
OPT = BASE / "optimization" / "n055_smooth_jerk80_rk4_formal_planning45"
PRE = BASE / "precontact"
COMP = BASE / "comparison"
CONTACT = BASE / "contact"
REACH = BASE / "reachability"


def read(path):
    with Path(path).open(encoding="utf-8") as handle:
        return json.load(handle)


def _first_qualified_metrics(directory, seed):
    source = directory / "joint" / f"seed_{seed:02d}"
    candidates = read(source / "candidate_replay_summary.json")["dynamic_ranking"]
    selected = next(item for item in candidates if item["planning_and_dynamic_qualified"])
    rank = selected["kinematic_rank"]
    return read(source / f"candidate_{rank:02d}" / "metrics.json")["metrics"]


def fig01_search():
    suite = read(OPT / "pso_suite_summary.json")["variants"]["joint"]["runs"]
    dynamic = read(PRE / "n055_smooth_jerk80_rk4_formal_planning45" / "dynamic_suite_summary.json")
    seeds = dynamic["variants"]["joint"]["seeds"]
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 3.4))
    ax = axes[0]
    points = ax.scatter([row["capture_time_s"] for row in suite],
                        [np.degrees(row["terminal_arm_angle_rad"]) for row in suite],
                        c=[row["seed"] for row in suite], cmap="viridis", vmin=0, vmax=9,
                        s=55)
    ax.annotate("seed 0", (suite[0]["capture_time_s"],
                np.degrees(suite[0]["terminal_arm_angle_rad"])), xytext=(5, 5),
                textcoords="offset points", fontsize=8)
    fig.colorbar(points, ax=ax, label="PSO seed", ticks=range(10), shrink=0.8)
    ax.set(xlabel="Capture time (s)", ylabel="Terminal arm angle (deg)")
    ax.grid(alpha=0.2)
    ax = axes[1]
    count = [row["jointly_qualified_candidate_count"] for row in seeds]
    ax.bar(range(10), count, color=COLORS["full"], width=0.72)
    ax.axhline(5, color="0.6", linestyle="--", linewidth=0.8)
    ax.set(xlabel="PSO seed", ylabel="Jointly qualified candidates / 5",
           xticks=list(range(10)), ylim=(0, 5.6))
    ax.grid(axis="y", alpha=0.2)
    finish(fig, "01_n055r_search_and_qualification")


def fig02_weights():
    groups = [
        ("1:1", PRE / "n061_weights_1_1_rk4_formal_planning45"),
        ("1:2", PRE / "n055_smooth_jerk80_rk4_formal_planning45"),
        ("2:1", PRE / "n061_weights_2_1_rk4_formal_planning45"),
    ]
    metrics = [
        ("base_cost", "Base cost (unweighted)"),
        ("alignment_cost", "Alignment cost (unweighted)"),
        ("maximum_base_angular_velocity_rad_s", "Peak base angular speed (rad/s)"),
        ("minimum_clearance_m", "Minimum clearance (m)"),
    ]
    rows = [[_first_qualified_metrics(path, seed) for seed in range(10)] for _, path in groups]
    fig, axes = plt.subplots(2, 2, figsize=(9.0, 6.2))
    for ax, (key, ylabel) in zip(axes.flat, metrics):
        for x, group in enumerate(rows):
            values = np.array([row[key] for row in group], dtype=float)
            ax.scatter(np.full(10, x) + np.linspace(-0.11, 0.11, 10), values,
                       s=22, alpha=0.65, color=["#0072B2", "#D55E00", "#009E73"][x])
            ax.hlines(values.mean(), x - 0.19, x + 0.19, linewidth=2.2,
                      color=["#0072B2", "#D55E00", "#009E73"][x])
        ax.set(xlim=(-0.5, 2.5), xticks=range(3), xticklabels=[g[0] for g in groups],
               xlabel="Base : alignment weight", ylabel=ylabel)
        ax.grid(axis="y", alpha=0.2)
    finish(fig, "02_n061_weight_sensitivity")


def fig03_fixed_time():
    data = read(COMP / "n070_fixed_time_rk4_formal_planning45" / "precontact_comparison.json")
    rows = data["methods"]
    metrics = [
        ("terminal_position_error_m", 1e3, "Terminal position error (mm)", 1.0),
        ("terminal_arm_angle_error_rad", 180 / np.pi, "Terminal arm angle error (deg)", 1.0),
        ("maximum_base_angular_velocity_rad_s", 1, "Peak base angular speed (rad/s)", None),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(10.3, 3.1))
    names = ["Optimized", "Fixed 0°", "Fixed 90°"]
    colors = [COLORS["full"], COLORS["fixed0"], COLORS["fixedpi2"]]
    for ax, (key, factor, ylabel, gate) in zip(axes, metrics):
        vals = [r[key] * factor for r in rows]
        ax.bar(range(3), vals, color=colors)
        if gate is not None:
            ax.axhline(gate, color="black", linestyle="--", linewidth=1, label="Acceptance gate")
            ax.legend(frameon=False, loc="upper left")
            ax.set_yscale("log")
        ax.set(xticks=range(3), xticklabels=names, ylabel=ylabel)
        ax.tick_params(axis="x", rotation=18)
        ax.grid(axis="y", alpha=0.2)
    finish(fig, "03_n070_fixed_time_diagnostic")


def fig04_reachability():
    files = [
        ("Fixed 0°", ["n071_fixed0_diagnostic_100ms.json"], COLORS["fixed0"]),
        ("Fixed 90°", [
            "n072_fixedpi2_diagnostic_100ms.json",
            "n072_fixedpi2_diagnostic_20ms_8p6_9p5.json",
            "n072_fixedpi2_diagnostic_2ms_branch_8p62_8p78.json",
            "n072_fixedpi2_diagnostic_2ms_gate_9p4_9p5.json",
        ], COLORS["fixedpi2"]),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.6))
    for ax, (label, names, color) in zip(axes, files):
        candidates = {}
        for name in names:
            for item in read(REACH / name)["candidates"]:
                candidates[round(item["capture_time_s"], 6)] = item
        position = np.array([r["terminal_position_error_m"] * 1000 for r in candidates.values()])
        shape = np.array([np.degrees(r["terminal_arm_angle_error_rad"]) for r in candidates.values()])
        ax.scatter(position, shape, s=11, alpha=0.65, color=color, rasterized=True)
        ax.axvline(1.0, color="0.4", linestyle="--", linewidth=0.8)
        ax.axhline(0.75, color="0.4", linestyle="--", linewidth=0.8)
        ax.set(xscale="log", yscale="log", xlabel="Terminal position error (mm)",
               ylabel="Terminal arm angle error (deg)")
        ax.text(0.04, 0.96, f"{label}: 0/{len(candidates)} feasible", transform=ax.transAxes,
                ha="left", va="top", fontsize=9)
        ax.grid(alpha=0.18)
    finish(fig, "04_n071_n072_sampled_reachability")


def fig05_ablation():
    data = read(COMP / "n073_controller_ablation_rk4_formal_planning45" / "controller_ablation_summary.json")
    rows = data["runs"]
    by = {(r["seed"], r["variant"]): r for r in rows}
    variants = ["full", "no-shape", "no-base-reaction"]
    colors = [COLORS[x] for x in variants]
    fig, axes = plt.subplots(1, 3, figsize=(10.8, 3.4))
    ax = axes[0]
    values = [data["variant_counts"][v]["qualified_count"] for v in variants]
    ax.bar(range(3), values, color=colors)
    ax.set(xticks=range(3), xticklabels=["Full", "No shape", "No base"],
           ylabel="Accepted runs / 10", ylim=(0, 11))
    ax.tick_params(axis="x", rotation=18)
    ax = axes[1]
    for x, variant in enumerate(variants):
        y = [np.degrees(by[s, variant]["terminal_arm_angle_error_rad"]) for s in range(10)]
        ax.scatter(np.full(10, x) + np.linspace(-0.13, 0.13, 10), y,
                   color=COLORS[variant], alpha=0.7, s=20)
    ax.axhline(1.0, color="black", linestyle="--", linewidth=1)
    ax.set(xticks=range(3), xticklabels=["Full", "No shape", "No base"],
           ylabel="Terminal arm angle error (deg)", yscale="log")
    ax.tick_params(axis="x", rotation=18)
    ax = axes[2]
    keys = [
        ("maximum_base_angular_velocity_rad_s", "Peak speed"),
        ("rms_base_angular_velocity_rad_s", "RMS speed"),
        ("maximum_base_orientation_drift_rad", "Drift"),
    ]
    for x, (key, _) in enumerate(keys):
        improvement = [100 * (by[s, "no-base-reaction"][key] - by[s, "full"][key]) /
                       by[s, "no-base-reaction"][key] for s in range(10)]
        ax.scatter(np.full(10, x) + np.linspace(-0.14, 0.14, 10), improvement,
                   s=22, color=COLORS["full"], alpha=0.7)
        ax.hlines(np.median(improvement), x - 0.19, x + 0.19,
                  color=COLORS["full"], linewidth=2)
    ax.axhline(0, color="0.4", linewidth=0.8)
    ax.set(xticks=range(3), xticklabels=[name for _, name in keys],
           ylabel="Full vs no-base improvement (%)")
    ax.tick_params(axis="x", rotation=18)
    for ax in axes:
        ax.grid(axis="y", alpha=0.18)
    finish(fig, "05_n073_ablation_all_seeds")


def fig06_precontact_trace():
    path = PRE / "n073_controller_ablation_rk4_formal_planning45" / "seed_00"
    fig, axes = plt.subplots(2, 2, figsize=(9.0, 6.0), sharex=True)
    variants = ["full", "no-shape", "no-base-reaction"]
    labels = ["Full", "No shape", "No base reaction"]
    directories = {"full": "full", "no-shape": "no_shape",
                   "no-base-reaction": "no_base_reaction"}
    for variant, label in zip(variants, labels):
        with np.load(path / directories[variant] / "trace.npz") as trace:
            t = trace["time"]
            err = np.degrees(np.abs((trace["arm_angle_rad"] - trace["desired_arm_angle_rad"]
                                     + np.pi) % (2 * np.pi) - np.pi))
            series = [
                trace["position_error_m"] * 1000,
                err,
                np.linalg.norm(trace["base_twist"][:, 3:6], axis=1),
                trace["minimum_clearance_m"] * 1000,
            ]
            for ax, y in zip(axes.flat, series):
                ax.plot(t, y, color=COLORS[variant], linewidth=1.0, label=label)
    for ax, ylabel in zip(axes.flat, ["Position error (mm)", "Arm angle error (deg)",
                                       "Base angular speed (rad/s)", "Clearance (mm)"]):
        ax.set(ylabel=ylabel, xlabel="Time since approach start (s)")
        ax.grid(alpha=0.17)
    axes[0, 0].legend(frameon=False, loc="upper left")
    finish(fig, "06_n073_seed00_precontact_trace")


def fig07_prechecks():
    data = read(CONTACT / "n100_n101_precheck.json")
    force = data["n100_contact_wrench"]
    step = data["n101_normal_admittance"]
    params = step["parameters"]
    natural = np.sqrt(params["stiffness_n_m"] / params["virtual_mass_kg"])
    t = np.linspace(0, 0.3, 301)
    analytic = step["analytic_steady_offset_m"] * (1 - (1 + natural * t) * np.exp(-natural * t))
    fig, axes = plt.subplots(1, 2, figsize=(8.7, 3.3))
    axes[0].bar(["Expected weight", "Measured support"],
                [force["box_force_world_n"][2] - force["support_force_error_n"],
                 force["box_force_world_n"][2]], color=["0.55", COLORS["full"]])
    axes[0].set(ylabel="Vertical force (N)")
    axes[0].tick_params(axis="x", rotation=14)
    axes[1].plot(t, analytic * 1000, color=COLORS["full"], label="Analytical reconstruction")
    axes[1].axhline(step["analytic_steady_offset_m"] * 1000, color="0.4", linestyle="--",
                    linewidth=0.8, label="Steady offset")
    axes[1].axvline(step["two_percent_settling_time_s"], color=COLORS["no-shape"],
                    linestyle=":", linewidth=1, label="Reported 2% settling")
    axes[1].set(xlabel="Time after 3 N step (s)", ylabel="Normal offset (mm)")
    axes[1].legend(frameon=False)
    for ax in axes:
        ax.grid(axis="y", alpha=0.18)
    finish(fig, "07_n100_n101_contact_prechecks")


CONTACT_RUNS = [
    ("rigid", "n102_rigid_authoritative", "Rigid"),
    ("admittance", "n103_admittance_authoritative", "Admittance + shape"),
    ("admittance-no-shape", "n104_admittance_no_shape_authoritative", "Admittance - shape"),
]


def fig08_contact_traces():
    fig, axes = plt.subplots(2, 2, figsize=(9.5, 6.0), sharex=True)
    for variant, directory, label in CONTACT_RUNS:
        with np.load(CONTACT / directory / "trace.npz") as trace:
            t = trace["time"]
            force = trace["measured_normal_force_n"]
            detection = trace["contact_detected"]
            offset = trace["command_normal_offset_m"] * 1000
            penetration = trace["penetration_m"] * 1000
            axes[0, 0].plot(t, force, color=COLORS[variant], label=label, linewidth=1)
            axes[0, 1].plot(t, offset, color=COLORS[variant], linewidth=1)
            axes[1, 0].plot(t, penetration, color=COLORS[variant], linewidth=1)
            axes[1, 1].step(t, detection, where="post", color=COLORS[variant], linewidth=1)
            if variant == "admittance":
                axes[0, 0].plot(t, trace["desired_normal_force_n"], color="black",
                                linestyle="--", linewidth=1, label="Desired force")
    labels = ["Normal contact force (N)", "Commanded normal offset (mm)",
              "Penetration (mm)", "Contact detected (0/1)"]
    for ax, label in zip(axes.flat, labels):
        ax.set(ylabel=label, xlabel="Time after contact handoff (s)")
        ax.grid(alpha=0.18)
    axes[0, 0].legend(frameon=False, loc="upper center",
                      bbox_to_anchor=(1.08, 1.19), ncol=4, fontsize=7)
    axes[1, 1].set(ylim=(-0.07, 1.07))
    finish(fig, "08_n102_n104_contact_time_series")


def fig09_contact_metrics():
    data = read(CONTACT / "n102_n104_authoritative_comparison.json")["runs"]
    series = [
        ("peak_normal_force_n", "Peak force"),
        ("normal_force_impulse_ns", "Force impulse"),
        ("maximum_base_angular_speed_rad_s", "Peak base speed"),
        ("robot_contact_angular_impulse_about_initial_com_norm_nms", "Angular impulse"),
        ("maximum_penetration_m", "Penetration"),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 3.6))
    x = np.arange(len(series))
    for idx, (variant, _, label) in enumerate(CONTACT_RUNS):
        values = [data[variant][key] / data["rigid"][key] for key, _ in series]
        axes[0].bar(x + (idx - 1) * 0.24, values, width=0.23,
                    label=label, color=COLORS[variant])
    axes[0].axhline(1, color="0.35", linewidth=0.8)
    axes[0].set(xticks=x, xticklabels=[name for _, name in series],
                ylabel="Ratio to rigid baseline")
    axes[0].tick_params(axis="x", rotation=23)
    axes[0].legend(frameon=False, fontsize=7)
    rmse = [data[variant]["steady_force_rmse_n"] for variant, _, _ in CONTACT_RUNS]
    axes[1].bar(range(3), rmse, color=[COLORS[variant] for variant, _, _ in CONTACT_RUNS])
    desired = read(CONTACT / "n103_admittance_authoritative" / "metrics.json")["effective_contact_config"]["force_control"]["desired_normal_force_n"]
    fraction = read(CONTACT / "n103_admittance_authoritative" / "metrics.json")["effective_contact_config"]["acceptance"]["maximum_steady_force_rmse_fraction"]
    axes[1].axhline(desired * fraction, color="black", linestyle="--", linewidth=1,
                    label="Admittance gate")
    axes[1].set(xticks=range(3), xticklabels=["Rigid*", "Adm. + shape", "Adm. - shape"],
                ylabel="Steady force RMSE (N)")
    axes[1].tick_params(axis="x", rotation=18)
    axes[1].legend(frameon=False)
    for ax in axes:
        ax.grid(axis="y", alpha=0.18)
    finish(fig, "09_n102_n104_contact_metrics")
