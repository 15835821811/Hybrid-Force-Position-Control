# FPMFC Figure Plan

> 2026-09-16：旧目标几何生成的 A 系列图仅保留追溯用途。下列 N 系列来自新位置 `C=[0.95,0.14,0.95] m`、边长 `0.30 m` 和正确端面姿态，是当前正式图源。

| ID | Type | Description | Data Source / Output | Priority | Status |
|---|---|---|---|---|---|
| Fig. N1 | Three-panel optimization figure | 10-seed PSO convergence, final objective and feasibility, `T_c-psi_f` solution distribution | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/fig_optimization_process.pdf` | HIGH | DONE |
| Fig. N2 | Eight-panel dynamics figure | End-effector tracking, position/orientation errors, arm shape, base translation/attitude/angular speed, clearance | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/fig_tracking_error_base_drift.pdf` | HIGH | DONE |
| Fig. N3 | Animated process view | Isometric free-floating capture process with target, grasp point, arm and base motion | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/selected_capture_isometric.gif` | HIGH | DONE |
| Fig. N4 | Storyboard | Four representative frames for static inspection | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/selected_capture_storyboard.png` | MEDIUM | DONE |
| Fig. N5 | Three-panel joint kinematics | Actual joint position, velocity, and numerically differentiated acceleration for J1--J7 | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/five_view_joint_kinematics/joint_position_velocity_acceleration.pdf` | HIGH | DONE |
| Video N1 | Synchronized five-view dashboard | Isometric/front/right/top/rear views plus live J1--J7 position, velocity, and acceleration curves | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v2_selected/five_view_joint_kinematics/selected_capture_five_view_joint_kinematics.mp4` | HIGH | DONE |
| Fig. N6 | Six-second smoothing comparison | Pre/post command acceleration, command jerk envelope, and actual acceleration envelope for 5.5--7.2 s | `output/fpmfc/analysis/adaptive_cube_c095_014_095_face_aligned_v6_smooth_jerk80/six_second_smoothing_comparison.pdf` | HIGH | DONE |
| Video N2 | Smoothed synchronized five-view dashboard | Five views plus q/dq/ddq from the C²-saturated, jerk-limited selected trace | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v6_smooth_jerk80/five_view_joint_kinematics/selected_capture_five_view_joint_kinematics.mp4` | HIGH | DONE |
| GIF N2 | Smoothed capture animation | Isometric 8 s process from the verified smooth trace | `output/fpmfc/visualization/adaptive_cube_c095_014_095_face_aligned_v6_smooth_jerk80/single_view/selected_capture_isometric.gif` | HIGH | DONE |

All final figures are generated from saved formal artifacts. Smoke figures contain `smoke` in their path and must not be used as evidence. The authoritative input/output hashes and checks are recorded in `full_process_visualization_manifest.json` and `visualization_manifest.json` beside the final figures.
