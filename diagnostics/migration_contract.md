# Simscape-to-MuJoCo migration contract

| Concern | Simscape source | MuJoCo destination |
|---|---|---|
| System | Free-floating satellite plus one Flexiv Rizon 4s | Same physical scope |
| Base | `base_link_0`, 500 kg, diagonal inertia 20.8333 kg·m² | Free joint, same mass/inertia, 0.5 m cube |
| Arm | `base_link`, `link1` … `link7` | Same serial body tree and collision meshes |
| Joint coordinates | Seven revolute joints; home `[0,-40,0,90,0,40,0]°` | Same axes, limits and home angles in radians |
| Gravity | Orbital/zero-gravity configuration | `gravity="0 0 0"` |
| Original input | Prescribed joint motion in degrees | Seven bounded direct torque motors |
| Source sample time | 1 ms fixed step | 2 ms physics step required by V6 execution layer |
| Task control | Damped inverse-Jacobian trajectory blocks | One 7-variable weighted constrained QP at 50 Hz |
| Free-base coupling | Generalized Jacobian exposed by source | `v_base=-Mbb^-1 Mba qdot_arm` from live MuJoCo mass matrix |
| Safety | No complete signed-distance controller in source | Exact MuJoCo mesh/box/sphere distance barriers |
| Execution | Simscape computed torque under prescribed motion | 20 ms command ramp and 500 Hz model-based torque servo |
| Evidence | Scope/display signals | Replayable NPZ, JSON metrics, SHA-256 manifest and independent replay |

The interface change from prescribed motion to direct torque is intentional: it is required to reproduce the reference V6 two-rate execution semantics rather than merely convert CAD geometry. The controller eliminates the six unactuated base accelerations from the full mass matrix before computing the seven arm torques.

The source model has no second arm, continuum section, 17-dimensional planning state, or 67 actuators. Those reference-only features are not synthesized. The waypoint generator retains the V6 template, draw order, randomization ranges and minimum-jerk timing, with only its workspace center translated to the Flexiv reachable set.
