# Notation and assumptions

World frame W is inertial over the local microgravity simulation. B is robot
base, T the measured target geometric frame, G its fixed grasp frame, F the
flange, E the tool interface. T_AB maps coordinates from B to A. R_AB is its
rotation and p_AB the position of B's origin expressed in A. The target COM is
not generally T's origin. Angular velocities in formulas are spatial/world
unless explicitly tagged body. MuJoCo free-joint translational velocity is world
and rotational velocity local body; Jacobians are used for point twists.

q_j has seven relative joint angles. Generalized configuration comprises two
free bodies plus seven joints (nq21,nv19,nu7). Only joint torque is commanded.
The target has no actuator; base has no thruster/reaction wheel. Dynamics use
positive physical inertias, fixed original geometry and compliant pad contact;
event weld is a numerical equivalent interface, not a demonstrated gripper.

The inherited "20 mm tool" label denotes the N206 **+20 mm axial extension**,
not a 20 mm distance from the flange origin to the physical front face.
The original pad front is at flange z=-0.2 mm; after the extension the physical
front is at z=19.8 mm, comprising a 9.8 mm spacer and 10 mm pad. The separately
calibrated interface site E retains its original mounting transform, translated
by the same extension; it must not be replaced by the physical-face center.
These definitions are recorded in
[N206 installation configuration](../../configs/n206_geometry_capture.yaml)
and [selected geometry](../../output/fpmfc/n206_geometry_capture/selected_design.json),
fields parameters.increment_m, parameters.pad_front_z_m and T_flange_tool.
N209 changes none of those quantities. The main manuscript's "20 mm" refers
to this inherited design label; the physical-face trace correctly gives 19.8 mm.

s in [0,1] is path progress; code progress is u=8s in virtual seconds. Thus
sdot=rate/8 away from the endpoint. Filter states remain defined after the path
ends, while the path itself stays fixed relative to the target. Endpoint
consistency requires the relative path's first and second jets to vanish;
this is checked, not inferred by clipping a nonzero derivative.

The state estimator stores geometric-origin position p_T, orientation R_WT,
world velocity v_T, world angular velocity omega_T and a12x12 local left-error
covariance ordered [delta p,delta theta,delta v,delta omega]. Pose stamps,
delivery/control times and mode history are distinct. No covariance entry is
a certified error bound. Directional3sigma and empirical coverage have neither
a joint3-D99.7% interpretation nor an arbitrary-disturbance guarantee.

pi=[m,h_x,h_y,h_z,Ixx,Iyy,Izz,Ixy,Ixz,Iyz] uses first moment h=m c and inertia
about T. This repository's final three ordering differs from some cited papers;
use momentum_regressor.py's tensor converter, not positional copying. pi_est
is the shadow estimate; pi_ctrl is the fixed prior for all N209 methods.

P is system linear momentum; H_O is angular momentum about one fixed world
origin O; H_C is angular momentum about total COM. H_O=H_C+C cross P. The
locked inertia I_L(q) relates common angular velocity to H_C when all relative
velocities vanish. P/H and wrench/energy residuals retain their original units.

Proof assumptions are conditional mathematical hypotheses, not automatically
validated properties of a noisy discretized simulation. In particular, exact
state/error bounds, exact predictive dynamics, no unmodeled external wrench,
ideal internal connections and a feasible future action need separate evidence.
