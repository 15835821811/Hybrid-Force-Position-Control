# S00 contract semantics and legacy boundary

These are compatibility contracts, not a new controller or a certified sensor
error model. The executable definitions are in
`v6_mujoco/system_capture/contracts.py` and the mappings in `adapters.py`.

|Object|Meaning and boundary|
|---|---|
|FrameConvention|Active column-vector `R_parent_from_child`; point linear velocity then angular velocity; force then moment; SI. A spatial-twist linear component must pass through the explicit point conversion.|
|RobotObservation|The old free-joint velocity is mixed: linear velocity in W, angular velocity in B. These are separate named arrays. `base_twist_world()` rotates the angular component. The inverse legacy mapping preserves the original stored arrays without a numerical round trip. Base pose uses wxyz quaternion order.|
|SensorPacket|Robot/F/T sample time, packet arrival and control time are the original servo time. Each delayed target geometry observation carries its own sample time and the actual packet delivery tick. Generation time was not separately logged and remains null. Contact/F/T are declared sensor channels, not additional actuation.|
|StateEstimate|Target visible geometry origin, not COM; p/R/v/w in W. Covariance error order is dp, left world rotation error, dv, domega. `bias_bound` records the old guard's additive positional/orientation allowances; zero entries for velocity/angular rate mean no extra allowance was supplied by that legacy interface, not a certified zero physical bias. `process_mode` records the legacy filter's held propagation mode; S00 does not correct its model.|
|ParameterEstimate|pi uses mass, first moments and inertia about target geometry origin. The inherited covariance is in the legacy scaled coordinates and its scope explicitly says local EIV approximation. Rank is finite-data rank, physical is the legacy consistency check, and feedback_used is separate from estimation.|
|MotionReference|World flange pose and point derivatives; s is the old virtual coordinate divided by the fixed 8-second template coordinate range. This is not physical elapsed time divided by the approach deadline. The timestamp and progress correspond to generation of the held task reference, with validity through the next task tick. It is absent after latch.|
|ControlProposal|Seven torques, legacy phase/status, current named planning-distance constraints and latch request. A terminal refusal has actuation_valid=false and SIM_ABORT; its placeholder tau must not be applied. Full HQP rows, joint/derivative bounds and solver records remain in the immutable historical hqp_diagnostics and source code, rather than being redefined by S00. No new recovery law is supplied.|
|ScenarioSpec|Hardware, evaluation-only truth, prior, sensor configuration, algorithm, task summary and numerical clocks are distinct top-level namespaces. `algorithm.legacy_mission` is an explicitly named lossless compatibility aggregate; it retains duplicated task/numerical/filter fields required by the unchanged constructor. It contains no actual target truth or scenario identifier. S00 does not silently rewrite that old constructor interface.|
|ControllerSetup|The controller receives only public prior and the compatibility algorithm configuration. Factories refuse a ScenarioSpec. Recursive checks reject declared truth/scenario/future-packet keys. This is a tested software boundary, not an OS sandbox or a claim against arbitrary malicious Python code.|
|RunManifest/PhaseResult|Execution uses frozen content identities; implementation, model scope, reproduction, task, benefit and admission remain separate. Final Git SHA is registered outside tracked content.|

Arrays and nested mappings are deeply snapshotted. Immutable byte-backed NumPy
arrays prevent later estimator updates, including the old invalid-t0 covariance
alias, from changing saved unified records. JSON decoding reruns schema
validation. Legacy replay inputs and historical snapshot files remain unchanged.

`paper_compat_srs` and `paper_compat_reserved` deliberately have no runnable
model/controller. They cannot fall back to Flexiv or inherit its performance.
S01 is the next proposed source-reproduction stage, subject to new authorization.
