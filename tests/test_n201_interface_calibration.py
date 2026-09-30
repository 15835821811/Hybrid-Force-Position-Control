"""Physical pose conventions and the actual-load qualification gate."""

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from v6_mujoco.postgrasp_calibration.common import ROOT, read, relative_rotation_error
from v6_mujoco.postgrasp_calibration.load_tests import fixture
from v6_mujoco.postgrasp.physics import constraint_jacobian


def test_common_rigid_rotation_and_frozen_nonidentity_reference():
    A=Rotation.from_rotvec([.1,.3,-.2]).as_matrix()
    reference=Rotation.from_rotvec([.2,-.1,.4]).as_matrix()
    B=A@reference
    Q=Rotation.from_rotvec([1,.2,-.4]).as_matrix()
    assert relative_rotation_error(Q@A,Q@B,reference)<1e-12
    changed=B@Rotation.from_rotvec([0,np.deg2rad(.05),0]).as_matrix()
    assert abs(relative_rotation_error(A,changed,reference)-np.deg2rad(.05))<1e-12
    assert relative_rotation_error(A,changed,reference)>0


def test_quaternion_sign_does_not_change_pose_error():
    q=Rotation.from_rotvec([.3,.2,.4]).as_quat()
    angle1=relative_rotation_error(np.eye(3),Rotation.from_quat(q).as_matrix(),np.eye(3))
    angle2=relative_rotation_error(np.eye(3),Rotation.from_quat(-q).as_matrix(),np.eye(3))
    assert abs(angle1-angle2)<1e-12


def test_site_fixture_has_zero_initial_constraint_residual():
    candidate=read(ROOT/"candidate_2_reference_correction.json")
    model,_,_=fixture(candidate["parameters"])
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    assert int(model.eq_objtype[0])==int(mujoco.mjtObj.mjOBJ_SITE)
    assert np.linalg.norm(data.efc_pos)<1e-9
    assert np.linalg.matrix_rank(constraint_jacobian(model,data))==6


def test_measured_load_gate_is_not_replaced_by_pose_success():
    suite=read(ROOT/"load_tests/candidate_2_balanced/results.json")
    assert all(r["pose_holding_passed"] for r in suite["results"])
    assert max(r["max_interface_load_fraction"] for r in suite["results"])>1
    assert all(r["maximum_net_force_moment"]<1e-10 for r in suite["results"])
