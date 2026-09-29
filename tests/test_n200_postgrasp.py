"""Independent weld load and physical-wrench checks before formal runs."""

import mujoco
import numpy as np


def test_small_model_known_force_and_moment():
    xml = """<mujoco>
      <option timestep="0.001" gravity="0 0 0" integrator="RK4"/>
      <worldbody>
        <body name="flange"><site name="postgrasp_tool_interface"/></body>
        <body name="tumbling_target">
          <freejoint name="target_free_joint"/>
          <inertial pos="0 0 0" mass="2" diaginertia="0.2 0.2 0.2"/>
          <site name="target_grasp_site"/>
        </body>
      </worldbody>
      <equality><weld name="postgrasp_latch" body1="flange" body2="tumbling_target"
        relpose="0 0 0 1 0 0 0" solref="0.02 1" solimp="0.95 0.99 0.001"
        torquescale="0.15"/></equality>
    </mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    target = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "tumbling_target")
    applied = np.array([5.0, -3.0, 2.0, 0.1, -0.2, 0.3])
    data.xfrc_applied[target] = applied
    for _ in range(500):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    assert data.nefc == 6
    J = np.asarray(data.efc_J).reshape(6,6)
    generalized = J.T @ data.efc_force
    jp, jr = np.zeros((3,6)), np.zeros((3,6))
    mujoco.mj_jac(model, data, jp, jr, data.xipos[target], target)
    physical = np.linalg.solve(np.vstack((jp,jr)).T, generalized)
    np.testing.assert_allclose(physical, -applied, atol=3e-2)
    assert np.linalg.norm(data.efc_pos[:3]) < 0.002
