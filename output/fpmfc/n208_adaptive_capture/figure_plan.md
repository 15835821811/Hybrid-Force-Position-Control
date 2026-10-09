# N208 figure plan (objective-driven; no paper manuscript is required)

All plots read the selected current N208 run and explicitly show its actual time
range and latch event. Failed horizons are not extended or spliced with history.
Generate vector PDF and 300 dpi PNG preview; captions live in the index/report.

| ID | Data and purpose | Caption scope |
|---|---|---|
| tracking | measured-plant flange vs estimated live reference, position/orientation errors | approach tracking; post-latch reference inactive |
| state_estimation | geometric-frame p/R/v/omega error and covariance margins | simulation sensor mode, delay; no real-vision claim |
| capture_and_load | relative capture position/rotation/twist, contact force, load fraction | fixed mechanical/design gates, event latch |
| detumbling | target world spin and target-base relative spin | fixed local final window and both distinct performance gates |
| momentum_energy | P and H drifts separately; kinetic/relative energy and input work | no subtraction of solver error from conservation gates |
| joints | seven joint positions, speeds and applied torques | physical model limits unchanged |
| geometry | original and new minimum safety distances | fixed N206 tool and geometry scope |
| identification | mass/COM/inertia vs truth (evaluation only), data rank and singular values | shadow/feedback labelled; rank excludes constraints/regularization |

Two videos maximum: one composite with five fixed views plus continuous base-body
side view, and one interface closeup. Labels include N208, true/estimated data,
sensor mode, actual time, phase and unvalidated hardware/tool assumptions.

Final additions: trajectory gives equal-aspect actual/reference XY and XZ projections; campaign_identification compares mass/COM/tensor error and raw data rank across all six frozen paired runs, preserving failed horizons in the caption. Total: 10 figures, each PDF plus PNG.
